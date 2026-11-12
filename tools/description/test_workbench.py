#!/usr/bin/env python3
"""Meaningful geometry-tool regressions: units, prismatic FK, collision, bad inputs."""
import hashlib,importlib.util,json,subprocess,sys,tempfile,unittest
from pathlib import Path

TOOLS=Path(__file__).resolve().parent
ROOT=TOOLS.parents[1]


class WorkbenchTests(unittest.TestCase):
    def setUp(self):
        parent=ROOT/'runs/workbench-tests';parent.mkdir(parents=True,exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(prefix='case-',dir=parent)
        self.path=Path(self.temp.name).resolve()
        assert self.path.is_relative_to(parent.resolve())
        import trimesh
        self.mesh=self.path/'box.stl'
        self.mesh.write_text(trimesh.creation.box().export(file_type='stl_ascii'),encoding='ascii')
        self.original_sha=hashlib.sha256(self.mesh.read_bytes()).hexdigest()

    def tearDown(self):
        assert self.path.is_relative_to((ROOT/'runs/workbench-tests').resolve())
        self.temp.cleanup()

    def run_tool(self,name,*args,expected=0):
        result=subprocess.run([sys.executable,str(TOOLS/name),*map(str,args)],capture_output=True,text=True)
        self.assertEqual(result.returncode,expected,result.stdout+result.stderr)
        return result

    def robot(self,kind='prismatic'):
        path=self.path/'robot.urdf'
        path.write_text(f'''<robot name="test"><link name="world"/>
<link name="moving"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
<link name="target"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
<joint name="slide" type="{kind}"><parent link="world"/><child link="moving"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="0" upper="0.2" effort="1" velocity="0.1"/></joint>
<joint name="target_mount" type="fixed"><parent link="world"/><child link="target"/><origin xyz="1 0 0.3"/></joint></robot>''',encoding='utf-8')
        return path

    def test_unknown_units_are_not_silently_metres(self):
        report=self.path/'raw.json'
        self.run_tool('inspect_mesh.py',self.mesh,'--json',report)
        data=json.loads(report.read_text())
        self.assertEqual(data['declared_units'],'unknown')
        self.assertIsNone(data['bounds_m'])
        self.assertEqual(data['component_count'],1)
        self.assertEqual(hashlib.sha256(self.mesh.read_bytes()).hexdigest(),self.original_sha)

    def test_explicit_units_and_rigid_derivation(self):
        spec=self.path/'spec.json';output=self.path/'derived.stl'
        spec.write_text(json.dumps({'input_sha256':self.original_sha,'units':'mm','components':'all',
            'transform_m':[[1,0,0,.1],[0,1,0,0],[0,0,1,0],[0,0,0,1]]}))
        self.run_tool('derive_mesh.py',self.mesh,'--spec',spec,'--output',output)
        meta=json.loads(output.with_suffix('.stl.json').read_text())
        self.assertAlmostEqual(meta['bounds_m'][0][0],.0995,places=8)
        self.assertEqual(hashlib.sha256(self.mesh.read_bytes()).hexdigest(),self.original_sha)
        self.run_tool('derive_mesh.py',self.mesh,'--spec',spec,'--output',output,expected=2)

    def test_reflection_and_wrong_source_are_rejected(self):
        spec=self.path/'bad.json';output=self.path/'bad.stl'
        base={'input_sha256':self.original_sha,'units':'mm','components':'all','transform_m':[[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]}
        spec.write_text(json.dumps(base))
        self.run_tool('derive_mesh.py',self.mesh,'--spec',spec,'--output',output,expected=2)
        self.assertFalse(output.exists())
        base['input_sha256']='0'*64;spec.write_text(json.dumps(base))
        self.run_tool('derive_mesh.py',self.mesh,'--spec',spec,'--output',output,expected=2)

    def test_prismatic_fk_and_not_run_collision_status(self):
        report=self.path/'fk.json'
        self.run_tool('inspect_urdf.py',self.robot(),'--joint','slide=0.2','--json',report)
        data=json.loads(report.read_text())
        t=data['link_poses_in_reference_frame']['moving']
        self.assertAlmostEqual(t[0][3],1)
        self.assertAlmostEqual(t[2][3],.2)
        self.assertEqual(data['collision']['status'],'NOT_RUN')
        self.assertEqual(data['physical_calibration'],'not_verified')

    def test_fcl_native_pair_or_explicit_unavailability(self):
        report=self.path/'contact.json'
        available=importlib.util.find_spec('fcl') is not None
        self.run_tool('inspect_urdf.py',self.robot(),'--json',report,'--pair','moving','target',
            '--sweep','slide','0','0.2','3','--require-clear',expected=1 if available else 2)
        data=json.loads(report.read_text())
        if available:
            self.assertEqual(data['collision']['primitive_shapes'],'native FCL')
            self.assertEqual(data['collision']['contacts'][-1]['sample'],2)
        else:
            self.assertEqual(data['collision']['status'],'NOT_RUN')

    def test_unsupported_joint_and_missing_asset_fail(self):
        report=self.path/'bad-urdf.json'
        self.run_tool('inspect_urdf.py',self.robot('floating'),'--json',report,expected=2)
        robot=self.robot();text=robot.read_text().replace('<box size="0.1 0.1 0.1"/>','<mesh filename="package://missing/not-there.stl"/>')
        robot.write_text(text)
        self.run_tool('inspect_urdf.py',robot,'--json',report,expected=2)


if __name__=='__main__':unittest.main()
