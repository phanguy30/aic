import xml.etree.ElementTree as ET
import yaml
import sys


def modify_sdf(sdf_path, config_path, output_path):
    # 1. Load the Configuration
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # 2. Parse the SDF (XML)
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    world = root.find("world")

    # --- RULE 1: Remove Specific Models ---
    if world is not None:
        # --- RULE 1: Remove Specific Models ---
        models_to_remove = config.get("remove_models", [])
        for model in world.findall("model"):
            if model.get("name") in models_to_remove:
                world.remove(model)
                print(f"Removed model: {model.get('name')}")

        # --- RULE 2: Remove Lights ---
        lights_to_remove = config.get("remove_lights", [])
        for light in world.findall("light"):
            if light.get("name") in lights_to_remove:
                world.remove(light)
                print(f"Removed light: {light.get('name')}")

        # --- RULE 3: Add overhead spot light (z=12, radius≈12) ---
        add_spot = config.get("add_spot_light", False)
        if add_spot:
            spot_light = ET.SubElement(
                world,
                "light",
                attrib={
                    "name": "dome_light",
                    "type": "spot",
                },
            )

            pose = ET.SubElement(spot_light, "pose")
            pose.text = "0 0 12 0 0 0"

            direction = ET.SubElement(spot_light, "direction")
            direction.text = "0 0 -1"

            intensity = ET.SubElement(spot_light, "intensity")
            intensity.text = "1"

            diffuse = ET.SubElement(spot_light, "diffuse")
            diffuse.text = "1 1 1 1"

            specular = ET.SubElement(spot_light, "specular")
            specular.text = "0.1 0.1 0.1 1"

            spot = ET.SubElement(spot_light, "spot")
            inner = ET.SubElement(spot, "inner_angle")
            inner.text = "0.8"  # ~46 degrees
            outer = ET.SubElement(spot, "outer_angle")
            outer.text = "1.57"  # ~90 degrees, radius≈12 at z=12
            falloff = ET.SubElement(spot, "falloff")
            falloff.text = "1"

            print("Added overhead spot light 'dome_light'")

        # --- RULE 4: Adjust Light Intensity ---
        intensity_multiplier = config.get("light_intensity_multiplier", 1.0)
        enclosure_intensity_multiplier = config.get(
            "enclosure_intensity_multiplier", 1.0
        )
        print(f"Light intensity multiplier: {intensity_multiplier} ")
        print(f"Enclosure intensity multiplier: {enclosure_intensity_multiplier} ")
        if intensity_multiplier != 1.0 or enclosure_intensity_multiplier != 1.0:
            for light in world.findall("light"):
                multiplier = intensity_multiplier
                light_name = light.get("name")

                if light_name == "enclosure_light":
                    multiplier = enclosure_intensity_multiplier

                if multiplier == 1.0:
                    continue

                intensity_tag = light.find("intensity")
                if intensity_tag is not None and intensity_tag.text:
                    existing_val = float(intensity_tag.text)
                    new_val = existing_val * multiplier
                    intensity_tag.text = f"{new_val:.2f}"
                    print(
                        "Updated light"
                        f" '{light_name}' intensity to {intensity_tag.text}"
                    )

    # 3. Save the Modified SDF
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"Modified SDF saved to: {output_path}")


if __name__ == "__main__":
    # Example usage: python3 filter_sdf.py /tmp/aic.sdf config.yaml /tmp/aic_modified.sdf
    if len(sys.argv) != 4:
        print("Usage: python3 filter_sdf.py <input.sdf> <config.yaml> <output.sdf>")
        sys.exit(1)

    modify_sdf(sys.argv[1], sys.argv[2], sys.argv[3])
