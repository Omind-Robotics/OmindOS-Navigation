import hashlib,json,sys,tempfile,shutil
from pathlib import Path
sys.path.insert(0,'/opt/omindos/scripts')
import quadruped_controller,robot_parameter_api,robot_workbench,torch,mujoco,numpy
root=Path('/opt/omindos')
for module in [quadruped_controller,robot_parameter_api,robot_workbench]: assert module.__file__.endswith('.so')
modules=list((root/'scripts').glob('*.so'));assert len(modules)==19
bad=[str(p) for p in root.rglob('*') if p.is_file() and p.suffix in ('.py','.pyc','.pyo','.c','.cpp','.h','.map')]
assert not bad,bad
assert not list(root.rglob('*.customer-record.json'))
checks=['19_native_modules','no_backend_source_or_generated_C','private_customer_records_excluded']
with tempfile.TemporaryDirectory() as d:
    p=Path(d);(p/'REFERENCE.json').write_text('{}')
    try: quadruped_controller.PinnedPolicy(p)
    except ValueError as e: assert 'manifest mismatch' in str(e)
    else: raise AssertionError('tampered manifest accepted')
    checks.append('tampered_model_manifest_rejected')
    shutil.copytree(root/'reference',p,dirs_exist_ok=True)
    target=p/'models/scene.xml';target.write_bytes(target.read_bytes()+b'\n')
    try: quadruped_controller.PinnedPolicy(p)
    except ValueError as e: assert 'asset mismatch' in str(e)
    else: raise AssertionError('tampered model asset accepted')
    checks.append('tampered_model_asset_rejected')
print(json.dumps({'status':'PASS','checks':checks,'native_modules':len(modules),'backend_source_included':False,'versions':{'python':sys.version.split()[0],'torch':torch.__version__,'mujoco':mujoco.__version__,'numpy':numpy.__version__}}))