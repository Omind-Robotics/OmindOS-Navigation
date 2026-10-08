from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile
root=Path('/home/ubuntu/omindos-build/preview3-20261008')
version='0.2.0-preview.5'
name='omindos-control-preview-'+version+'-linux-amd64'
pkg=root/'distribution'/name
if pkg.exists(): raise SystemExit('Refusing to overwrite a prepared distribution')
pkg.mkdir(parents=True)
for name in ['omindos','README.zh-CN.md','THIRD_PARTY.md','RELEASE_NOTES.md']:
    shutil.copy2(root/'source/release/control-preview'/name,pkg/name)
(pkg/'omindos').chmod(0o755)
shutil.copytree(root/'native-app/config',pkg/'config')
shutil.copytree(root/'native-app/LICENSES',pkg/'LICENSES')
for name in ['native-dynamics.json','native-ros-quadruped.json','VALIDATION.json']:
    shutil.copy2(root/name,pkg/name)
image='omindos/control-preview:'+version
imagefile=pkg/('omindos-control-preview-'+version+'-linux-amd64.image.tar.gz')
with imagefile.open('wb') as stream:
    save=subprocess.Popen(['docker','save',image],stdout=subprocess.PIPE)
    gzip=subprocess.run(['gzip','-1'],stdin=save.stdout,stdout=stream)
    save.stdout.close()
    if save.wait() or gzip.returncode: raise RuntimeError('Image export failed')
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as s:
        for block in iter(lambda:s.read(4*1024*1024),b''): h.update(block)
    return h.hexdigest()
imagehash=sha(imagefile)
(pkg/'SHA256SUMS-image.txt').write_text(imagehash+'  '+imagefile.name+'\n')
image_id=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()
(pkg/'IMAGE_ID.txt').write_text(image_id+'\n')
for filename,command in [('THIRD_PARTY_PACKAGES.txt',['dpkg-query','-W']),('THIRD_PARTY_PYTHON.json',['/opt/omindos-runtime/bin/python','-m','pip','list','--format=json'])]:
    with (pkg/filename).open('w') as stream:
        subprocess.run(['docker','run','--rm','--entrypoint',command[0],image]+command[1:],check=True,stdout=stream)
metadata={'version':version,'platform':'linux/amd64','distribution':'compiled_control_preview','source_commit':(root/'source-commit.txt').read_text().strip(),'image_id':image_id,'image_sha256':imagehash,'image_bytes':imagefile.stat().st_size,'backend_source_included':False,'hardware_validated':False,'jetson_arm64_included':False,'model':'original_MEVIUS2','model_commit':'4f09680bb575574377b903bbc971bdb2695d507b','private_customer_record_included':False,'files':{str(f.relative_to(pkg)):sha(f) for f in sorted(pkg.rglob('*')) if f.is_file()}}
(pkg/'MANIFEST.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
archive=root/'distribution'/(pkg.name+'.tar')
with tarfile.open(archive,'w') as tar: tar.add(pkg,arcname=pkg.name)
summary={**{k:v for k,v in metadata.items() if k!='files'},'filename':archive.name,'bytes':archive.stat().st_size,'sha256':sha(archive),'download_url':'https://omindos.cn/downloads/navigation/v'+version+'/'+archive.name,'publication_status':'approved_preview'}
(root/'distribution/RELEASE_SUMMARY.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
(root/'distribution/SHA256SUMS-release.txt').write_text(summary['sha256']+'  '+archive.name+'\n')
print(json.dumps(summary,ensure_ascii=False),flush=True)