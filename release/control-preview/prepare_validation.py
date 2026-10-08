"""Point the existing regression suite at compiled modules in an isolated copy."""
from pathlib import Path
import shutil

root=Path('/build')
target=root/'native-tests'
target.mkdir(exist_ok=True)
for name in ['config','scripts','web']:
    shutil.copytree(root/'native-app'/name,target/name,dirs_exist_ok=True)
shutil.copytree(root/'source/tests',target/'tests',dirs_exist_ok=True)
for path in (target/'tests').glob('test_*.py'):
    text=path.read_text()
    if path.name=='test_robot_parameter_api.py':
        start=text.index('SPEC = ');end=text.index('\n\n',start)
        text=text[:start]+"import sys\nsys.path.insert(0,str(ROOT/'scripts'))\nimport robot_parameter_api as API"+text[end:]
    elif path.name=='test_customer_profile.py':
        start=text.index('SPEC = ');end=text.index('\n\n',start)
        text=text[:start]+"sys.path.insert(0,str(ROOT/'scripts'))\nimport check_customer_profile as MODULE"+text[end:]
        text=text.replace("[sys.executable, str(ROOT / \"scripts/check_customer_profile.py\"),", "[sys.executable, '-c', 'import check_customer_profile;raise SystemExit(check_customer_profile.main())',")
    elif path.name=='test_joint_interface.py':
        text=text.replace('[sys.executable,str(script),',"[sys.executable,'-c','import quadruped_worker;quadruped_worker.main()',")
    path.write_text(text)
text=(root/'source/scripts/check_robot_profile_ros.py').read_text()
text=text.replace("[sys.executable,str(ROOT/'scripts/robot_profile_runtime.py'),", "[sys.executable,'-c','import robot_profile_runtime;robot_profile_runtime.main()',")
# Private test driver lives next to compiled modules and is never copied to a release.
(target/'scripts/check_robot_profile_ros.py').write_text(text)
shutil.copy2(root/'source/scripts/check_quadruped_ros2.py',target/'scripts/check_quadruped_ros2.py')
print('Prepared unchanged assertions with native module entry points')
