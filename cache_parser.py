# originally from https://gist.github.com/fxtentacle/d3002f2c4084b9758c4228d5aedb276a
import numpy as np
import json
from pathlib import Path
import argparse

import open3d as o3d

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument('-p','--param_dir', help="the name of the 'param' dir relative to the basepath",default='param')
arg_parser.add_argument('-c','--cache_dir', help="the name of the 'cache' dir relative to the basepath", default='cache')
arg_parser.add_argument('basepath',default=".",nargs='?', help="the path containing the property.rvproj file")
args = arg_parser.parse_args()
print(args)

base_path_val=args.basepath
param_dir_val=args.param_dir
cache_dir_val=args.cache_dir

basepath=Path(base_path_val)
param_dir=Path(basepath,param_dir_val)
cache_dir=Path(basepath,cache_dir_val)

print(param_dir.absolute())

def from_json(json,*keys):
    current = json
    for key in keys:
        if current is not None:
            try:
                current = current[key]
            except:
                current = None
        else:
            break
    return current

def from_json_or_default(json,*vals,default=None):
    reso = None
    for val in vals:
        if reso is not None:
            break
        reso = from_json(json, *val)
    if reso is None:
        reso=default
    return reso

# load the project JSON to get the depth scale
with Path(basepath,'property.rvproj').absolute().open(mode='rt') as f:
    project = json.load(f)
depth_scale = project['scan_param']['depth_scale']
depth_height = from_json_or_default(project,['scan_param','depth_height'],['scan_param','ir_height'],default=400)
depth_width = from_json_or_default(project,['scan_param','depth_width'],['scan_param','ir_width'],default=640)
print(depth_scale)

# load projection matrix from 2D to 3D
Q = np.fromfile(str(Path(param_dir,'Q.bin').absolute()), count=4*4, dtype=np.float32).reshape((4,4,))
print('Q', Q)



def load_frame(filename):
    fname=str(filename)
    # load depth map, convert to float, scale
    frame = np.fromfile(fname, dtype=np.uint16).reshape((depth_height,depth_width,)).astype(np.float32) * depth_scale

    # make [x,y,z,1] vectors
    xs = np.arange(0,depth_width).astype(np.float32)
    ys = np.arange(0,depth_height).astype(np.float32)
    xs = np.tile(xs.reshape((1,depth_width)),(depth_height,1,))
    ys = np.tile(ys.reshape((depth_height,1)),(1,depth_width,))
    data = np.stack([xs,ys,frame,np.ones_like(frame),],-1)

    # flatten 2D image to 1D pixel collection and drop pixels without depth
    data = data.reshape((-1,4))
    data = data[data[:,2] > 0]

    # apply projection matrix Q to get ray direction, then scale by z
    zs = data[:,2:3]
    data = np.matmul(Q.reshape((1,4,4,))[:,:3,:4], data.reshape((-1,4,1,)))
    data = data[:,0:3,0] * zs / data[:,2:3,0]

    # RevoScan appears to work with inverted Y and Z axis
    data = np.stack([data[:,0],-data[:,1],-data[:,2]], -1)

    # check that the object size is reasonable
    print('extends', np.max(data,0)-np.min(data,0))

    if True:
        # load the transformation matrix for this frame
        T = np.fromfile(fname.replace('.dph','.inf'), offset=0x10, count=4*4, dtype=np.float64).reshape((4, 4,))
        print('T', T)
        # convert point cloud to [x,y,z,1] and multipla and reduce to [x/w,y/w,z/w]
        data = np.concatenate([data,np.ones_like(data[:,0:1])],-1)
        data = np.matmul(T.reshape((1,4,4,)), data.reshape((-1,4,1,)))
        data = data[:,0:3,0] / data[:,3:4,0]

    return data

depth_files = cache_dir.absolute().glob("*.dph")

# load demo frames and merge them into one point cloud
data = np.concatenate(
    list(map(load_frame,depth_files)),
#     [
#     load_frame('./cache/frame_000_0000.dph'),
#     load_frame('./cache/frame_000_0002.dph'),
#     load_frame('./cache/frame_000_0004.dph'),
#     load_frame('./cache/frame_000_0006.dph'),
#     load_frame('./cache/frame_000_0008.dph'),
#     load_frame('./cache/frame_000_0010.dph'),
#     load_frame('./cache/frame_000_0012.dph'),
#     load_frame('./cache/frame_000_0014.dph'),
#     load_frame('./cache/frame_003_0000.dph'),
#     load_frame('./cache/frame_003_0002.dph'),
#     load_frame('./cache/frame_003_0004.dph'),
#     load_frame('./cache/frame_003_0020.dph'),
#     load_frame('./cache/frame_004_0001.dph'),
#     load_frame('./cache/frame_004_0003.dph'),
#     load_frame('./cache/frame_008_0001.dph'),
#     load_frame('./cache/frame_008_0107.dph'),
# ], 
0)

# display result
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(data)
vwe = o3d.visualization.VisualizerWithEditing()
#o3d.visualization.gui.Application.instance.initialize()
#window = o3d.visualization.gui.Application.instance.create_window("Add Spheres Example", 1024, 768)
#scene = o3d.visualization.gui.SceneWidget()
#scene.scene = o3d.visualization.rendering.Open3DScene(window.renderer)
#window.add_child(vwe.create_window())
#scene.scene.add_geometry(pcd)
vwe.create_window()
vwe.add_geometry(pcd)
vwe.run()
#o3d.visualization.gui.Application.instance.run()

#o3d.visualization.draw_geometries_with_editing([pcd])
