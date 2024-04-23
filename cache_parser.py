# originally from https://gist.github.com/fxtentacle/d3002f2c4084b9758c4228d5aedb276a
import numpy as np
import json

base_path="."
param_dir="param"
cache_dir="cache"

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
with open('./property.rvproj','rt') as f:
    project = json.load(f)
depth_scale = project['scan_param']['depth_scale']
depth_height = from_json_or_default(project,['scan_param','depth_height'],['scan_param','ir_height'],default=400)
depth_width = from_json_or_default(project,['scan_param','depth_width'],['scan_param','ir_width'],default=640)
print(depth_scale)

# load projection matrix from 2D to 3D
Q = np.fromfile('./param/Q.bin', count=4*4, dtype=np.float32).reshape((4,4,))
print('Q', Q)



def load_frame(filename):
    # load depth map, convert to float, scale
    frame = np.fromfile(filename, dtype=np.uint16).reshape((depth_height,depth_width,)).astype(np.float32) * depth_scale

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
        T = np.fromfile(filename.replace('.dph','.inf'), offset=0x10, count=4*4, dtype=np.float64).reshape((4, 4,))
        print('T', T)
        # convert point cloud to [x,y,z,1] and multipla and reduce to [x/w,y/w,z/w]
        data = np.concatenate([data,np.ones_like(data[:,0:1])],-1)
        data = np.matmul(T.reshape((1,4,4,)), data.reshape((-1,4,1,)))
        data = data[:,0:3,0] / data[:,3:4,0]

    return data

# load demo frames and merge them into one point cloud
data = np.concatenate([
    load_frame('./cache/frame_000_0000.dph'),
    load_frame('./cache/frame_000_0002.dph'),
    load_frame('./cache/frame_000_0004.dph'),
    load_frame('./cache/frame_000_0006.dph'),
    load_frame('./cache/frame_000_0008.dph'),
    load_frame('./cache/frame_000_0010.dph'),
    load_frame('./cache/frame_000_0012.dph'),
    load_frame('./cache/frame_000_0014.dph'),
    load_frame('./cache/frame_003_0000.dph'),
    load_frame('./cache/frame_003_0002.dph'),
    load_frame('./cache/frame_003_0004.dph'),
    load_frame('./cache/frame_003_0020.dph'),
    load_frame('./cache/frame_004_0001.dph'),
    load_frame('./cache/frame_004_0003.dph'),
    load_frame('./cache/frame_008_0001.dph'),
    load_frame('./cache/frame_008_0107.dph'),
], 0)

# display result
import open3d as o3d
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(data)
o3d.visualization.draw_geometries([pcd])
