import yaml
import sys
from pxr import Usd, Sdf


def modify_usda(usda_path, config_path, output_path):
    # 1. Load the Configuration
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # 2. Parse the USD for querying (Stage)
    stage = Usd.Stage.Open(usda_path)
    if not stage:
        print(f"Error: Could not open USD file {usda_path}.")
        sys.exit(1)

    # --- RULE 1: Find Target Paths ---
    # We expect the YAML to have a dictionary map called 'replace_meshes'
    replacements = config.get("replace_meshes", {})
    target_names = list(replacements.keys())
    found_paths = {}

    # Traverse the stage ONCE to find all target paths efficiently
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in target_names:
            found_paths[name] = prim.GetPath()
            if len(found_paths) == len(target_names):
                break  # Found everything, stop traversing early

    # 3. Open the Sdf Layer for physical file manipulation
    main_layer = Sdf.Layer.FindOrOpen(usda_path)
    if not main_layer:
        print(f"Error: Could not open Sdf Layer for {usda_path}.")
        sys.exit(1)

    # --- RULE 2: Replace Meshes ---
    for target_name, mesh_file in replacements.items():
        if target_name not in found_paths:
            print(f"Warning: Could not find '{target_name}' in {usda_path}. Skipping.")
            continue

        usd_target_path = found_paths[target_name]

        # Form Sdf.Path objects
        dst_path = Sdf.Path(usd_target_path)
        src_path = Sdf.Path(f"/{target_name}")
        parent_path = dst_path.GetParentPath()

        # Load the replacement layer
        mesh_layer = Sdf.Layer.FindOrOpen(mesh_file)
        if not mesh_layer:
            print(f"Error: Could not load replacement file '{mesh_file}'. Skipping.")
            continue

        # Delete the old prim
        parent_spec = main_layer.GetPrimAtPath(parent_path)
        if parent_spec and target_name in parent_spec.nameChildren:
            del parent_spec.nameChildren[target_name]
            print(f"Removed old prim: {target_name} at {usd_target_path}")

        # Copy the new prim over
        Sdf.CopySpec(mesh_layer, src_path, main_layer, dst_path)
        print(f"Injected new mesh from {mesh_file} into {usd_target_path}")

    # --- RULE 3: Set dome light radius (if it exists) ---
    dome_light_radius = config.get("dome_light_radius")
    if dome_light_radius is not None:
        dome_light_prim = None
        # Traverse the stage to find the dome light by its name
        for prim in stage.Traverse():
            if prim.GetName() == "dome_light":
                dome_light_prim = prim
                break

        if dome_light_prim:
            # The prim is a UsdLux.Light. We need to set its radius.
            radius_attr = dome_light_prim.GetAttribute("inputs:radius")
            if radius_attr:
                radius_attr.Set(float(dome_light_radius))
                print(
                    f"Set 'inputs:radius' of '{dome_light_prim.GetName()}' to {dome_light_radius}"
                )
        else:
            print("Warning: 'dome_light' not found in the stage.")

    # 4. Save the Modified USD
    main_layer.Export(output_path)
    print(f"Modified USD saved to {output_path}")


if __name__ == "__main__":
    # Example usage: python3 usda_modifier.py aic_enclosure.usda config.yaml aic_enclosure_updated.usda
    if len(sys.argv) != 4:
        print(
            "Usage: python3 usda_modifier.py <input.usda> <config.yaml> <output.usda>"
        )
        sys.exit(1)

    modify_usda(sys.argv[1], sys.argv[2], sys.argv[3])
