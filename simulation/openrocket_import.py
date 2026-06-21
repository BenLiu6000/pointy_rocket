from __future__ import annotations

import base64
import csv
import io
import math
import re
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path


MAX_BODY_EXPORT_BYTES = 2_000_000


def import_rocket_body(file_name, encoded_content):
    raw = base64.b64decode(encoded_content, validate=True)
    if len(raw) > MAX_BODY_EXPORT_BYTES:
        raise ValueError("Rocket body export is too large.")
    suffix = Path(file_name).suffix.lower()
    if suffix in (".csv", ".tsv"):
        return import_component_table(raw.decode("utf-8", errors="replace"), suffix)
    if suffix not in (".ork", ".rkt", ".cdx1", ".xml"):
        raise ValueError("Upload a .ork, .rkt, .cdx1, .csv, or .tsv body export.")
    xml_bytes = read_body_xml(raw)
    return import_body_xml(xml_bytes)


def import_component_table(content, suffix):
    delimiter = "\t" if suffix == ".tsv" else ","
    rows = list(csv.DictReader(io.StringIO(content), delimiter=delimiter))
    if not rows:
        raise ValueError("Body table export has no rows.")
    headers = {header.lower().strip(): header for header in rows[0].keys() if header}
    name_key = find_header(headers, ("component", "name", "part"))
    mass_key = find_header(headers, ("mass", "weight"))
    position_key = find_header(headers, ("cg", "center of gravity", "centre of gravity", "position", "location"))
    components = []
    for row in rows[:80]:
        name = row.get(name_key, "Component").strip()[:80] or "Component"
        mass = scaled_number(row.get(mass_key, ""), mass_scale(mass_key))
        position = scaled_number(row.get(position_key, ""), length_scale(position_key))
        if mass > 0 and math.isfinite(position):
            components.append({"name": name, "mass": mass, "position": position})
    if not components:
        raise ValueError("Body table needs component, mass, and CG/position columns.")
    return response_from_components(components)


def read_body_xml(raw):
    if raw[:2] == b"PK":
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            names = [name for name in archive.namelist() if name.lower().endswith((".ork", ".rkt", ".cdx1", ".xml"))]
            if not names:
                raise ValueError("Body archive does not contain a design XML file.")
            info = archive.getinfo(names[0])
            if info.file_size > MAX_BODY_EXPORT_BYTES:
                raise ValueError("Body design XML is too large.")
            return archive.read(info)
    return raw


def import_body_xml(xml_bytes):
    try:
        root = ET.fromstring(xml_bytes)
    except ET.ParseError as error:
        raise ValueError(f"Invalid rocket body XML: {error}") from error
    collected = []
    walk_components(root, 0.0, collected)
    if not collected:
        collected = collect_named_components(root)
    if not collected:
        return import_flat_xml(root)
    rocket_length = max(item["front"] + item["length"] for item in collected)
    components = estimated_components(collected, rocket_length)
    if not components:
        return import_flat_xml(root)
    response = response_from_components(components)
    response["length"] = rocket_length
    response["estimatedMassCount"] = sum(1 for item in components if item.get("estimated"))
    radius = body_radius(collected)
    if radius > 0:
        response["radius"] = radius
    return response


def import_flat_xml(root):
    components = []
    for element in root.iter():
        mass = named_number(element, ("mass", "weight", "componentmass"))
        position = named_number(element, ("cg", "centerofgravity", "centreofgravity", "position", "location"))
        if mass > 0 and math.isfinite(position):
            components.append(
                {
                    "name": element.attrib.get("name") or direct_text(element, "name", local_name(element.tag))[:80],
                "mass": mass,
                "position": position,
            }
        )
    if not components:
        raise ValueError("No component masses and CG positions found. Export a component table with mass and CG columns.")
    return response_from_components(components[:80])


def collect_named_components(root):
    collected = []
    collect_named_components_at(root, 0.0, collected)
    return collected


def collect_named_components_at(element, parent_front, collected):
    cursor = 0.0
    for child in list(element):
        child_tag = local_name(child.tag).lower()
        if child_tag in ("name", "length", "material", "massoverrides", "position", "finish", "color"):
            continue
        name = component_name(child)
        if not physical_component(child, name):
            collect_named_components_at(child, parent_front + cursor, collected)
            continue
        length = best_length(child)
        offset = component_offset(child, cursor, length)
        front = parent_front + offset
        mass = component_mass(child, length)
        cg = component_cg(child, length)
        collected.append({"name": name, "mass": mass, "front": front, "length": length, "cg": cg, "radius": component_radius(child)})
        collect_named_components_at(child, front, collected)
        cursor = max(cursor, offset + max(length, 0.02))


def physical_component(element, name):
    tag = local_name(element.tag).lower()
    if tag in ("name", "length", "material", "massoverrides", "position", "finish", "color"):
        return False
    if direct_child(element, "name") is None and tag in ("entry", "property", "preset"):
        return False
    if any(direct_child(element, key) is not None for key in ("length", "massoverrides", "material", "outerradius", "radius", "thickness", "mass")):
        return True
    if math.isfinite(attribute_float(element, ("mass", "weight", "componentmass", "packedmass"), math.nan)):
        return True
    return name != local_name(element.tag) and len(list(element)) > 0


def estimated_components(collected, rocket_length):
    components = []
    fallback_mass = 0.01
    for item in collected[:80]:
        estimated = item["mass"] <= 0
        mass = item["mass"] if item["mass"] > 0 else fallback_mass
        position = max(0.0, rocket_length - (item["front"] + item["cg"]))
        components.append({"name": item["name"], "mass": mass, "position": position, "estimated": estimated})
    return components


def body_radius(collected):
    radii = [item["radius"] for item in collected if item.get("radius", 0.0) > 0]
    return max(radii) if radii else 0.0


def walk_components(element, parent_front, collected, inherited_radius=0.0):
    cursor = 0.0
    for child in list(element):
        if not has_component_shape(child):
            walk_components(child, parent_front, collected, inherited_radius)
            continue
        length = best_length(child)
        offset = component_offset(child, cursor, length)
        front = parent_front + offset
        name = component_name(child)
        radius = component_radius(child) or inherited_radius
        mass = component_mass(child, length, radius)
        cg = component_cg(child, length)
        if length > 0 or mass > 0:
            collected.append({"name": name, "mass": mass, "front": front, "length": max(0.0, length), "cg": cg, "radius": radius})
        subcomponents = direct_child(child, "subcomponents")
        if subcomponents is not None:
            walk_components(subcomponents, front, collected, radius)
        cursor = max(cursor, offset + max(0.0, length))


def has_component_shape(element):
    if any(direct_child(element, name) is not None for name in ("length", "massoverrides", "material", "radius", "outerradius", "thickness", "mass")):
        return True
    return math.isfinite(attribute_float(element, ("mass", "weight", "componentmass", "packedmass"), math.nan))


def best_length(element):
    length = first_direct_float(element, ("length", "packedlength", "packedLength"), 0.0)
    if length > 0:
        return length
    fore = first_direct_float(element, ("foreRadius", "foreradius", "forediameter"), 0.0)
    aft = first_direct_float(element, ("aftRadius", "aftradius", "aftdiameter"), 0.0)
    if fore > 0 or aft > 0:
        return max(fore, aft, 0.02)
    return 0.02


def component_radius(element):
    radius = first_direct_float(element, ("outerradius", "outerRadius", "radius", "aftradius", "foreRadius", "packedradius", "packedRadius"), 0.0)
    if radius > 0:
        return radius
    diameter = first_direct_float(element, ("outerdiameter", "outerDiameter", "diameter", "aftdiameter", "forediameter"), 0.0)
    return diameter / 2 if diameter > 0 else 0.0


def component_mass(element, length, inherited_radius=0.0):
    overrides = direct_child(element, "massoverrides")
    if overrides is not None:
        mass = override_mass(overrides)
        if mass > 0:
            return mass
    mass = first_direct_float(element, ("mass", "packedmass", "stageseparationmass"), 0.0)
    if mass <= 0:
        mass = attribute_float(element, ("mass", "weight", "componentmass", "packedmass"), 0.0)
    if mass > 0:
        return mass
    recovery_mass = parachute_mass(element)
    if recovery_mass > 0:
        return recovery_mass
    density = material_density(element)
    if density <= 0:
        return 0.0
    radius = component_radius(element) or inherited_radius
    thickness = first_direct_float(element, ("thickness", "wallthickness", "wallThickness"), 0.0)
    inner_radius = first_direct_float(element, ("innerradius", "innerRadius"), 0.0)
    if length > 0 and radius > 0:
        if thickness > 0:
            inner_radius = max(0.0, radius - thickness)
        if inner_radius > 0 and inner_radius < radius:
            return math.pi * (radius**2 - inner_radius**2) * length * density
        if thickness > 0:
            return 2 * math.pi * radius * length * thickness * density
    if length > 0 and radius > 0:
        return math.pi * radius**2 * length * density / 3
    return 0.0


def parachute_mass(element):
    if local_name(element.tag).lower() != "parachute":
        return 0.0
    diameter = first_direct_float(element, ("diameter",), 0.0)
    canopy_density = material_density(element)
    mass = math.pi * (diameter / 2) ** 2 * canopy_density if diameter > 0 and canopy_density > 0 else 0.0
    line_count = int(first_direct_float(element, ("linecount",), 0.0))
    line_length = first_direct_float(element, ("linelength",), 0.0)
    line_material = direct_child(element, "linematerial")
    if line_count > 0 and line_length > 0 and line_material is not None:
        line_density = material_density_from(line_material)
        mass += line_count * line_length * line_density
    return mass


def override_mass(overrides):
    mass = direct_float(overrides, "mass", 0.0)
    if mass <= 0:
        mass = attribute_float(overrides, ("mass", "weight", "componentmass"), 0.0)
    if mass > 0:
        return mass
    for child in overrides.iter():
        if child is overrides:
            continue
        if local_name(child.tag).lower() in ("mass", "weight", "componentmass"):
            try:
                return float((child.text or "").strip())
            except ValueError:
                pass
    return 0.0


def component_cg(element, length):
    overrides = direct_child(element, "massoverrides")
    if overrides is not None:
        cg = direct_float(overrides, "cg", math.nan)
        if not math.isfinite(cg):
            cg = attribute_float(overrides, ("cg", "centerofgravity", "centreofgravity"), math.nan)
        if math.isfinite(cg):
            return cg
    cg = attribute_float(element, ("cg", "centerofgravity", "centreofgravity"), math.nan)
    if math.isfinite(cg):
        return cg
    return max(0.0, length / 2)


def component_offset(element, cursor, length):
    child = direct_child(element, "position")
    if child is None:
        child = direct_child(element, "axialoffset")
    if child is None:
        return cursor
    value = parse_number(child.text, cursor)
    position_type = (child.attrib.get("type") or child.attrib.get("method") or "top").lower()
    if position_type == "bottom":
        return cursor + value - length
    if position_type == "middle":
        return cursor + value - length / 2
    if position_type == "absolute":
        return value
    return cursor + value


def named_number(element, names):
    value = attribute_float(element, names, math.nan)
    if math.isfinite(value):
        return value
    for name in names:
        value = direct_float(element, name, math.nan)
        if math.isfinite(value):
            return value
    return math.nan


def attribute_float(element, names, default):
    lowered = {key.lower().replace("_", "").replace("-", ""): value for key, value in element.attrib.items()}
    for name in names:
        value = lowered.get(name.lower().replace("_", "").replace("-", ""))
        if value is None:
            continue
        return parse_number(value, default)
    return default


def material_density(element):
    material = direct_child(element, "material")
    if material is None:
        return 0.0
    return material_density_from(material)


def material_density_from(material):
    density = attribute_float(material, ("density",), math.nan)
    if math.isfinite(density):
        return density
    for key in ("density", "materialdensity"):
        value = direct_float(material, key, math.nan)
        if math.isfinite(value):
            return value
    if material.text:
        try:
            parts = material.text.replace(",", " ").split()
            for part in parts:
                value = float(part)
                if value > 0:
                    return value
        except ValueError:
            pass
    return 0.0


def response_from_components(components):
    total_mass = sum(component["mass"] for component in components)
    if total_mass <= 0:
        raise ValueError("Imported component mass is zero.")
    return {
        "components": components,
        "dryMass": total_mass,
        "dryCg": sum(component["mass"] * component["position"] for component in components) / total_mass,
    }


def find_header(headers, candidates):
    for candidate in candidates:
        for header_key, header in headers.items():
            if candidate in header_key:
                return header
    raise ValueError(f"Missing column: {candidates[0]}.")


def scaled_number(value, scale):
    return parse_number(value, math.nan) * scale


def mass_scale(header):
    lower = header.lower()
    return 0.001 if "(g" in lower or lower.endswith(" g") else 1.0


def length_scale(header):
    lower = header.lower()
    if "(mm" in lower or lower.endswith(" mm"):
        return 0.001
    if "(cm" in lower or lower.endswith(" cm"):
        return 0.01
    return 1.0


def direct_child(element, name):
    for child in list(element):
        if local_name(child.tag).lower() == name.lower():
            return child
    return None


def direct_text(element, name, default):
    child = direct_child(element, name)
    if child is None or child.text is None:
        return default
    return child.text.strip() or default


def component_name(element):
    return (element.attrib.get("name") or direct_text(element, "name", local_name(element.tag)))[:80]


def element_number(element, name, default):
    value = direct_float(element, name, math.nan)
    if math.isfinite(value):
        return value
    return attribute_float(element, (name,), default)


def direct_float(element, name, default):
    child = direct_child(element, name)
    if child is None or child.text is None:
        return default
    return parse_number(child.text, default)


def parse_number(value, default):
    if value is None:
        return default
    matches = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", str(value))
    if not matches:
        return default
    try:
        return float(matches[-1])
    except ValueError:
        return default


def first_direct_float(element, names, default):
    for name in names:
        value = direct_float(element, name, math.nan)
        if math.isfinite(value):
            return value
    return default


def local_name(tag):
    return tag.rsplit("}", 1)[-1]
