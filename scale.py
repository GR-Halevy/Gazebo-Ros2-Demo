from lxml import etree

# scales geometry, poses and meshes of a model file in SDF format
# more customized plugins in gazebo should still be modified manually
# such as ackermann steering, these values need to be set correctly for nice odometry
input_file = "models/X1/model.sdf"
output_file = "scaled_model.sdf"
scale_factor = 0.4


def scale_float(val):
    return str(float(val) * scale_factor)

def scale_pose(pose_text):
    values = pose_text.strip().split()
    if len(values) >= 3:
        scaled_pos = [scale_float(v) for v in values[:3]]
        rot = values[3:] if len(values) > 3 else []
        return " ".join(scaled_pos + rot)
    return pose_text




with open(input_file, "r") as file:
    sdf_content = file.read()
root = etree.fromstring(sdf_content.encode())

for tag in root.xpath(".//visual | .//collision"):
    geometry = tag.find("geometry")
    if geometry is None:
        continue

    mesh = geometry.find("mesh")
    if mesh is not None:
        scale_elem = mesh.find("scale")
        if scale_elem is None:
            scale_elem = etree.SubElement(mesh, "scale")
        scale_elem.text = f"{scale_factor} {scale_factor} {scale_factor}"

    for primitive in ["box", "cylinder", "sphere"]:
        shape = geometry.find(primitive)
        if shape is not None:
            if primitive == "box":
                size = shape.find("size")
                if size is not None:
                    size.text = " ".join(scale_float(v) for v in size.text.split())
            elif primitive == "cylinder":
                radius = shape.find("radius")
                length = shape.find("length")
                if radius is not None:
                    radius.text = scale_float(radius.text)
                if length is not None:
                    length.text = scale_float(length.text)
            elif primitive == "sphere":
                radius = shape.find("radius")
                if radius is not None:
                    radius.text = scale_float(radius.text)

    pose_elem = tag.find("pose")
    if pose_elem is not None:
        pose_elem.text = scale_pose(pose_elem.text)

for link in root.xpath(".//link"):
    pose_elem = link.find("pose")
    if pose_elem is not None:
        pose_elem.text = scale_pose(pose_elem.text)

with open(output_file, "w") as file:
    file.write(etree.tostring(root, pretty_print=True, encoding="unicode"))

print(f"Scaled model saved to {output_file}")
