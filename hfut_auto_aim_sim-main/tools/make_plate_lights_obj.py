#!/usr/bin/env python3

import argparse
from collections import defaultdict
from pathlib import Path


LIGHT_MATERIAL = "RM2019裁判系统装甲模块AM02:color:255:35:0"
PLATE_MATERIAL = "RM2019裁判系统装甲模块AM02:black_spray_paint:15:15:15"
DIGIT_MATERIAL = "RM2019裁判系统装甲模块AM02:color:255:255:255"


def parse_obj(path, light_material):
    vertices = []
    texcoords = []
    normals = []
    current_material = None
    light_faces = []
    body_indices = set()

    with Path(path).open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                vertices.append(tuple(float(x) for x in line.split()[1:4]))
            elif line.startswith("vt "):
                texcoords.append(tuple(float(x) for x in line.split()[1:]))
            elif line.startswith("vn "):
                normals.append(tuple(float(x) for x in line.split()[1:4]))
            elif line.startswith("usemtl "):
                current_material = line.strip()[7:]
            elif line.startswith("f "):
                tokens = line.split()[1:]
                indices = [int(token.split("/")[0]) - 1 for token in tokens]
                body_indices.update(indices)
                if current_material == light_material:
                    light_faces.append(tokens)

    return vertices, texcoords, normals, light_faces, body_indices


def bounds(points):
    mins = [min(point[i] for point in points) for i in range(3)]
    maxs = [max(point[i] for point in points) for i in range(3)]
    return mins, maxs


def light_components(vertices, light_faces):
    parent = list(range(len(light_faces)))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(a, b):
        a_root = find(a)
        b_root = find(b)
        if a_root != b_root:
            parent[b_root] = a_root

    coord_to_face = {}
    for face_index, face in enumerate(light_faces):
        for token in face:
            vertex_index = int(token.split("/")[0]) - 1
            coord = tuple(round(value, 5) for value in vertices[vertex_index])
            other_face = coord_to_face.get(coord)
            if other_face is None:
                coord_to_face[coord] = face_index
            else:
                union(face_index, other_face)

    components = defaultdict(list)
    for face_index in range(len(light_faces)):
        components[find(face_index)].append(face_index)
    return list(components.values())


def split_face_token(token):
    parts = token.split("/")
    vertex = int(parts[0])
    texcoord = int(parts[1]) if len(parts) > 1 and parts[1] else None
    normal = int(parts[2]) if len(parts) > 2 and parts[2] else None
    return vertex, texcoord, normal


def format_face(token, vertex_map, texcoord_map, normal_map):
    vertex, texcoord, normal = split_face_token(token)
    mapped_vertex = str(vertex_map[vertex])

    if texcoord is None and normal is None:
        return mapped_vertex
    if normal is None:
        return f"{mapped_vertex}/{texcoord_map[texcoord]}"
    if texcoord is None:
        return f"{mapped_vertex}//{normal_map[normal]}"
    return f"{mapped_vertex}/{texcoord_map[texcoord]}/{normal_map[normal]}"


def parse_simple_obj(path):
    vertices = []
    faces = []

    with Path(path).open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                vertices.append(tuple(float(x) for x in line.split()[1:4]))
            elif line.startswith("f "):
                faces.append([int(token.split("/")[0]) for token in line.split()[1:]])

    return vertices, faces


def write_box(f, next_index, x0, x1, y_inner, y_outer, z0, z1):
    if x1 <= x0 or y_outer <= y_inner or z1 <= z0:
        return next_index, 0

    box_vertices = [
        (x0, y_outer, z0),
        (x1, y_outer, z0),
        (x1, y_outer, z1),
        (x0, y_outer, z1),
        (x0, y_inner, z0),
        (x1, y_inner, z0),
        (x1, y_inner, z1),
        (x0, y_inner, z1),
    ]
    for vertex in box_vertices:
        f.write("v {:.9g} {:.9g} {:.9g}\n".format(*vertex))

    box_faces = [
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ]
    for face in box_faces:
        f.write(
            "f {} {} {}\n".format(*(next_index + vertex_index for vertex_index in face))
        )

    return next_index + len(box_vertices), len(box_faces)


def padded_rect(rect, margin, outer_x0, outer_x1, outer_z0, outer_z1):
    x0, x1, z0, z1 = rect
    return (
        max(outer_x0, x0 - margin),
        min(outer_x1, x1 + margin),
        max(outer_z0, z0 - margin),
        min(outer_z1, z1 + margin),
    )


def scale_rect_short_side(rect, scale):
    x0, x1, z0, z1 = rect
    if scale >= 1.0:
        return rect

    x_span = x1 - x0
    z_span = z1 - z0
    if x_span <= 0.0 or z_span <= 0.0:
        return rect

    if x_span <= z_span:
        center = 0.5 * (x0 + x1)
        half = 0.5 * x_span * scale
        return (center - half, center + half, z0, z1)

    center = 0.5 * (z0 + z1)
    half = 0.5 * z_span * scale
    return (x0, x1, center - half, center + half)


def resize_rect(rect, target_x_span, target_z_span):
    x0, x1, z0, z1 = rect
    x_center = 0.5 * (x0 + x1)
    z_center = 0.5 * (z0 + z1)
    x_span = target_x_span if target_x_span > 0.0 else x1 - x0
    z_span = target_z_span if target_z_span > 0.0 else z1 - z0
    return (
        x_center - 0.5 * x_span,
        x_center + 0.5 * x_span,
        z_center - 0.5 * z_span,
        z_center + 0.5 * z_span,
    )


def rect_contains(rect, x, z):
    x0, x1, z0, z1 = rect
    return x0 < x < x1 and z0 < z < z1


def write_plate_with_cutouts(
    f, next_index, outer_x0, outer_x1, y_inner, y_outer, outer_z0, outer_z1, holes
):
    x_edges = [outer_x0, outer_x1]
    z_edges = [outer_z0, outer_z1]
    for x0, x1, z0, z1 in holes:
        x_edges.extend([x0, x1])
        z_edges.extend([z0, z1])

    x_edges = sorted(set(round(edge, 6) for edge in x_edges))
    z_edges = sorted(set(round(edge, 6) for edge in z_edges))

    face_count = 0
    for x0, x1 in zip(x_edges, x_edges[1:]):
        if x1 <= x0:
            continue
        for z0, z1 in zip(z_edges, z_edges[1:]):
            if z1 <= z0:
                continue
            cx = 0.5 * (x0 + x1)
            cz = 0.5 * (z0 + z1)
            if any(rect_contains(hole, cx, cz) for hole in holes):
                continue
            next_index, box_faces = write_box(
                f, next_index, x0, x1, y_inner, y_outer, z0, z1
            )
            face_count += box_faces

    return next_index, face_count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_obj")
    parser.add_argument("output_obj")
    parser.add_argument("--light-material", default=LIGHT_MATERIAL)
    parser.add_argument("--plate-material", default=PLATE_MATERIAL)
    parser.add_argument(
        "--plate-y",
        type=float,
        default=171.564,
        help="Outer plate face y coordinate in source mesh millimeters.",
    )
    parser.add_argument(
        "--plate-thickness",
        type=float,
        default=15.0,
        help="Plate thickness in source mesh millimeters. Thickness extends inward.",
    )
    parser.add_argument(
        "--plate-width",
        type=float,
        default=135.0,
        help="Final armor plate x span in millimeters. Use <=0 to keep source width.",
    )
    parser.add_argument(
        "--plate-height",
        type=float,
        default=135.0,
        help="Final armor plate z span in millimeters. Use <=0 to keep source height.",
    )
    parser.add_argument(
        "--fill-light-bars",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Deprecated alias for --rect-light-bars.",
    )
    parser.add_argument(
        "--rect-light-bars",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Replace each original light-bar component with its minimum x-z bounding rectangle.",
    )
    parser.add_argument(
        "--light-fill-y-offset",
        type=float,
        default=0.0,
        help="Offset rectangular light bars outward from the plate face, in millimeters.",
    )
    parser.add_argument(
        "--light-depth",
        type=float,
        default=6.0,
        help="Embedded light-bar depth from the front face inward, in millimeters.",
    )
    parser.add_argument(
        "--light-hole-margin",
        type=float,
        default=0.2,
        help="Extra x-z clearance for plate cutouts around each light bar, in millimeters.",
    )
    parser.add_argument(
        "--light-short-side-scale",
        type=float,
        default=0.65,
        help="Scale factor for the narrow x-z side of each rectangular light bar.",
    )
    parser.add_argument(
        "--light-width",
        type=float,
        default=7.8,
        help="Final light-bar x span in millimeters. Use <=0 to fall back to short-side scale.",
    )
    parser.add_argument(
        "--light-height",
        type=float,
        default=50.0,
        help="Final light-bar z span in millimeters. Use <=0 to keep source height.",
    )
    parser.add_argument(
        "--fit-plate-x-to-lights",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Shrink the plate side edges to the outer x bounds of the light bars.",
    )
    parser.add_argument(
        "--digit-obj",
        default=None,
        help="Optional digit OBJ to merge into the simplified body mesh.",
    )
    parser.add_argument(
        "--include-digit",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Merge the digit mesh into this OBJ instead of loading it as a separate CadShape.",
    )
    parser.add_argument(
        "--digit-material",
        default=DIGIT_MATERIAL,
        help="Material used for the merged digit mesh.",
    )
    parser.add_argument(
        "--digit-thickness",
        type=float,
        default=2.0,
        help="Digit relief thickness in source mesh millimeters.",
    )
    parser.add_argument(
        "--digit-y-offset",
        type=float,
        default=4.0,
        help="Offset merged digit outer face from the plate face, in millimeters.",
    )
    parser.add_argument(
        "--flip-digit-faces",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Reverse digit face winding because the original digit mesh used ccw FALSE.",
    )
    args = parser.parse_args()
    if args.fill_light_bars is not None:
        args.rect_light_bars = args.fill_light_bars
    if args.digit_obj is None:
        args.digit_obj = str(Path(args.input_obj).with_name("armor_red4_digit.obj"))
    args.light_short_side_scale = max(0.01, min(args.light_short_side_scale, 1.0))

    vertices, texcoords, normals, light_faces, body_indices = parse_obj(
        args.input_obj, args.light_material
    )
    if not light_faces:
        raise RuntimeError(f"No faces found for material {args.light_material!r}")

    body_points = [vertices[i] for i in body_indices]
    body_min, body_max = bounds(body_points)

    used_vertex_indices = sorted(
        {
            split_face_token(token)[0]
            for face in light_faces
            for token in face
        }
    )
    used_texcoord_indices = sorted(
        {
            texcoord
            for face in light_faces
            for token in face
            for texcoord in [split_face_token(token)[1]]
            if texcoord is not None
        }
    )
    used_normal_indices = sorted(
        {
            normal
            for face in light_faces
            for token in face
            for normal in [split_face_token(token)[2]]
            if normal is not None
        }
    )
    index_map = {old_index: new_index for new_index, old_index in enumerate(used_vertex_indices, 1)}
    texcoord_map = {
        old_index: new_index
        for new_index, old_index in enumerate(used_texcoord_indices, 1)
    }
    normal_map = {
        old_index: new_index
        for new_index, old_index in enumerate(used_normal_indices, 1)
    }

    out = Path(args.output_obj)
    with out.open("w", encoding="utf-8", newline="\n") as f:
        f.write("# Low-cost armor body: 3D plate, front recessed light bars, covered rear, and raised digit.\n")
        f.write("# Generated by webots/tools/make_plate_lights_obj.py.\n")
        f.write("mtllib armor_red4.mtl\n\n")

        x0, y_outer, z0 = body_min[0], args.plate_y, body_min[2]
        x1, z1 = body_max[0], body_max[2]
        y_inner = y_outer - args.plate_thickness
        light_depth = min(max(args.light_depth, 0.0), args.plate_thickness)
        light_y_outer = y_outer + args.light_fill_y_offset
        light_y_inner = max(y_inner, light_y_outer - light_depth)
        recess_y_inner = min(y_outer, light_y_inner)

        light_rects = []
        for component in light_components(vertices, light_faces):
            component_points = [
                vertices[split_face_token(token)[0] - 1]
                for face_index in component
                for token in light_faces[face_index]
            ]
            comp_min, comp_max = bounds(component_points)
            light_rects.append((comp_min[0], comp_max[0], comp_min[2], comp_max[2]))

        plate_fit_rects = light_rects
        light_rects = [
            scale_rect_short_side(rect, args.light_short_side_scale)
            for rect in light_rects
        ]

        if args.fit_plate_x_to_lights:
            x0 = min(rect[0] for rect in plate_fit_rects)
            x1 = max(rect[1] for rect in plate_fit_rects)

        if args.plate_width > 0.0:
            x_center = 0.5 * (x0 + x1)
            x0 = x_center - 0.5 * args.plate_width
            x1 = x_center + 0.5 * args.plate_width
        if args.plate_height > 0.0:
            z_center = 0.5 * (z0 + z1)
            z0 = z_center - 0.5 * args.plate_height
            z1 = z_center + 0.5 * args.plate_height

        if args.light_width > 0.0 or args.light_height > 0.0:
            light_rects = [
                resize_rect(rect, args.light_width, args.light_height)
                for rect in plate_fit_rects
            ]

        light_holes = [
            padded_rect(rect, args.light_hole_margin, x0, x1, z0, z1)
            for rect in light_rects
        ]

        f.write("g SIMPLE_ARMOR_PLATE_REAR_COVER\n")
        f.write(f"usemtl {args.plate_material}\n")
        next_index, plate_face_count = write_box(
            f, 1, x0, x1, y_inner, recess_y_inner, z0, z1
        )

        f.write("\n")
        f.write("g SIMPLE_ARMOR_PLATE_FRONT_WITH_LIGHT_CUTOUTS\n")
        f.write(f"usemtl {args.plate_material}\n")
        next_index, front_plate_faces = write_plate_with_cutouts(
            f, next_index, x0, x1, recess_y_inner, y_outer, z0, z1, light_holes
        )
        plate_face_count += front_plate_faces
        f.write("\n")

        original_light_faces = 0
        if not args.rect_light_bars:
            f.write("g ORIGINAL_RED_LIGHT_BARS\n")
            f.write(f"usemtl {args.light_material}\n")
            for old_index in used_vertex_indices:
                f.write("v {:.9g} {:.9g} {:.9g}\n".format(*vertices[old_index - 1]))
            for old_index in used_texcoord_indices:
                texcoord = texcoords[old_index - 1]
                f.write("vt {}\n".format(" ".join(f"{value:.9g}" for value in texcoord)))
            for old_index in used_normal_indices:
                normal = normals[old_index - 1]
                f.write("vn {:.9g} {:.9g} {:.9g}\n".format(*normal))

            offset_map = {
                old: mapped + next_index - 1 for old, mapped in index_map.items()
            }
            for face in light_faces:
                f.write(
                    "f "
                    + " ".join(
                        format_face(token, offset_map, texcoord_map, normal_map)
                        for token in face
                    )
                    + "\n"
                )
            original_light_faces = len(light_faces)

        rect_light_faces = 0
        if args.rect_light_bars:
            f.write("\n")
            f.write("g RECTANGULAR_RED_LIGHT_BARS\n")
            f.write(f"usemtl {args.light_material}\n")
            for bar_x0, bar_x1, bar_z0, bar_z1 in light_rects:
                next_index, face_count = write_box(
                    f,
                    next_index,
                    bar_x0,
                    bar_x1,
                    light_y_inner,
                    light_y_outer,
                    bar_z0,
                    bar_z1,
                )
                rect_light_faces += face_count

        digit_faces_written = 0
        digit_path = Path(args.digit_obj)
        if args.include_digit and digit_path.exists():
            digit_vertices, digit_faces = parse_simple_obj(digit_path)
            if digit_vertices and digit_faces:
                digit_min, digit_max = bounds(digit_vertices)
                src_y_span = digit_max[1] - digit_min[1]
                digit_y_outer = y_outer + args.digit_y_offset
                digit_y_inner = digit_y_outer - args.digit_thickness

                f.write("\n")
                f.write("g MERGED_WHITE_DIGIT\n")
                f.write(f"usemtl {args.digit_material}\n")
                digit_offset = next_index - 1
                for x, y, z in digit_vertices:
                    if src_y_span > 0:
                        t = (y - digit_min[1]) / src_y_span
                        y = digit_y_inner + t * (digit_y_outer - digit_y_inner)
                    else:
                        y = digit_y_outer
                    f.write("v {:.9g} {:.9g} {:.9g}\n".format(x, y, z))
                for face in digit_faces:
                    if args.flip_digit_faces:
                        face = list(reversed(face))
                    f.write(
                        "f "
                        + " ".join(str(digit_offset + vertex_index) for vertex_index in face)
                        + "\n"
                    )
                next_index += len(digit_vertices)
                digit_faces_written = len(digit_faces)

    print(f"Wrote {out}")
    print(f"plate faces: {plate_face_count}")
    print(f"source light faces: {len(light_faces)}")
    print(f"original light faces written: {original_light_faces}")
    print(f"rectangular light faces: {rect_light_faces}")
    print(f"digit faces: {digit_faces_written}")
    print(f"total faces: {plate_face_count + original_light_faces + rect_light_faces + digit_faces_written}")


if __name__ == "__main__":
    main()
