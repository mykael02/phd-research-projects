"""Render the tested payload geometry as scientific illustrations, not hardware photos."""
import json
import xml.etree.ElementTree as ET
import mujoco
from PIL import Image
import extension_core as core


def main():
    out = core.ROOT/'results/mujoco_extension'
    out.mkdir(parents=True, exist_ok=True)
    entries = []
    for i, (mass, height) in enumerate([(.35, .14), (.65, .26), (1.15, .36)]):
        e = core.LoadedEngine('native', .00025, mass, height, .6)
        root = ET.fromstring(e.xml)
        root.find('.//geom[@name="body_shell"]').set('rgba', '.03 .48 .54 .22')
        root.find('.//geom[@name="front_panel"]').set('rgba', '.08 .15 .21 .12')
        # A visual-only, massless mast makes the rigid attachment legible.
        base = root.find('.//body[@name="base"]')
        ET.SubElement(base, 'geom', name='illustrative_payload_mast', type='box', pos=f'0 0 {height/2}', size=f'.006 .01 {height/2}', rgba='.45 .5 .55 1', contype='0', conaffinity='0')
        root.find('.//site[@name="body_com"]').set('rgba', '0 0 0 0')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        data = mujoco.MjData(model); mujoco.mj_forward(model, data)
        camera = mujoco.MjvCamera(); camera.azimuth=135; camera.elevation=-15; camera.distance=.9; camera.lookat[:]=[0, 0, .22]
        path = out/f'payload_model_{i+1}.png'
        with mujoco.Renderer(model, height=900, width=640) as renderer:
            renderer.update_scene(data, camera); Image.fromarray(renderer.render()).save(path)
        entries.append(dict(image=path.name, mass_kg=mass, height_m=height, loaded_mass_kg=e.p.body_mass,
                            combined_com_m=e.p.length, pitch_inertia_kg_m2=e.p.body_inertia))
    (out/'render_metadata.json').write_text(json.dumps({'models': entries, 'camera': {'azimuth':135,'elevation':-15,'distance':.9,'lookat':[0,0,.22]},
        'display_only': 'Transparent shell and massless noncolliding mast; inertial values and payload placement unchanged. Low payloads overlap the illustrative shell.'}, indent=2)+'\n')
    print('Rendered three native payload illustrations.')


if __name__ == '__main__': main()
