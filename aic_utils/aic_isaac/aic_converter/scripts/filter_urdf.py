import xml.etree.ElementTree as ET
import yaml
import sys
import os


def modify_urdf(urdf_path, config_path, output_path):
    # 1. Load the Configuration
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # 2. Parse the URDF (XML)
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # --- RULE 1: Remove all gazebo tags ---
    # Iterate over a copy of the list to safely remove items while iterating
    for gazebo in list(root.findall("gazebo")):
        root.remove(gazebo)
        print("Removed a gazebo tag")

    # --- RULE 2: Convert hyphens and slashes to underscores in link/joint names ---
    # Keep track of renamed links to update references
    renamed_links = {}

    # Update <link> definitions
    for link in root.findall("link"):
        name = link.get("name")
        if name:
            new_name = name.replace("-", "_").replace("/", "_")
            if new_name != name:
                link.set("name", new_name)
                renamed_links[name] = new_name
                print(f"Renamed link: '{name}' -> '{new_name}'")

    # Update <joint> references (parent and child)
    for joint in root.findall("joint"):
        j_name = joint.get("name")
        if j_name:
            new_j_name = j_name.replace("-", "_").replace("/", "_")
            if new_j_name != j_name:
                joint.set("name", new_j_name)
                print(f"Renamed joint: '{j_name}' -> '{new_j_name}'")

        mimic = joint.find("mimic")
        if mimic is not None:
            m_joint = mimic.get("joint")
            if m_joint:
                mimic.set("joint", m_joint.replace("-", "_").replace("/", "_"))

        parent = joint.find("parent")
        if parent is not None:
            p_link = parent.get("link")
            if p_link in renamed_links:
                parent.set("link", renamed_links[p_link])

        child = joint.find("child")
        if child is not None:
            c_link = child.get("link")
            if c_link in renamed_links:
                child.set("link", renamed_links[c_link])

    # --- RULE 3: Add dont_collapse="true" to fixed joints ---
    for joint in root.findall("joint"):
        if joint.get("type") == "fixed":
            joint.set("dont_collapse", "true")

    # --- RULE 4, 5, 6: Update Mesh Paths ---
    # Expecting config to have a 'package_map' dict for resolving package://
    package_map = config.get("package_map", {})
    file_prefix = config.get("file_prefix", "")

    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue

        original_filename = filename

        # Rule 4: Replace .glb with .dae
        if filename.endswith(".glb"):
            filename = filename.replace(".glb", ".dae")

        # Rule 5: Dynamic package source update
        if filename.startswith("package://"):
            # Remove prefix
            clean_path = filename.replace("package://", "")
            try:
                # Split into package name and relative path
                pkg_name, rel_path = clean_path.split("/", 1)
                if pkg_name in package_map:
                    base_path = package_map[pkg_name]
                    # Join and ensure forward slashes
                    new_path = os.path.join(base_path, rel_path).replace("\\", "/")
                    filename = new_path
                else:
                    print(
                        f"Warning: Package '{pkg_name}' not found in config 'package_map'."
                    )
            except ValueError:
                print(f"Warning: Could not parse package path '{filename}'")
        elif filename.startswith("file://"):
            filename = filename.replace("file://", file_prefix)

        # Rule 6: Handle spaces and hyphens in directory name (replace with underscores)
        filename = filename.replace(" ", "_").replace("-", "_")

        if filename != original_filename:
            mesh.set("filename", filename)
            print(f"Updated mesh path: {original_filename} -> {filename}")

    # 3. Save the Modified URDF
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"Modified URDF saved to: {output_path}")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python3 filter_urdf.py <input.urdf> <config.yaml> <output.urdf>")
        sys.exit(1)

    modify_urdf(sys.argv[1], sys.argv[2], sys.argv[3])
