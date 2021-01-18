import os
import subprocess

if __name__ == "__main__":
    rootdir = '/Users/datatang/Downloads/QA_3D_20201212'
    dirlist = os.listdir(rootdir)  # 列出文件夹下所有的目录与文件
    for i in range(0, len(dirlist)):
        path = os.path.join(rootdir, dirlist[i])
        if os.path.isdir(path):
        	org_path=os.path.join(path, 'four_lidar')
        	new_path=os.path.join(path, 'new_lidar')
        	if not os.path.exists(new_path):
        		os.makedirs(new_path) 
        	filelist = os.listdir(org_path)
        	for j in range(0, len(filelist)):
        		org_filename=os.path.join(org_path,filelist[j])
        		new_filename=os.path.join(new_path,filelist[j])
        		subprocess.run(["./transform",org_filename,new_filename])