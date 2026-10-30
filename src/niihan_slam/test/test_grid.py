import numpy as np
from scipy.spatial import cKDTree
from niihan_slam.grid import Grid,icp_planar

def test_unknown_is_preserved_and_obstacle_endpoint_not_cleared():
    g=Grid(.1,100);g.update(np.array([[2.,0.,.5],[4.,0.,.5],[1.,0.,3.]]),[0.,0.,.5],0.)
    data=g.data();assert data[50,70]==100;assert data[50,60]==0;assert data[50,80]==-1;assert data[50,90]==100;assert data[10,10]==-1

def test_registration_recovers_known_planar_transform():
    rng=np.random.default_rng(42);source=rng.uniform(-3,3,(2000,3));angle=.08;r=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]);target=source.copy();target[:,:2]=source[:,:2]@r.T+[.12,-.15]
    pose,rmse,overlap=icp_planar(source,cKDTree(target),[0.,0.,0.],max_distance=.5)
    assert np.allclose(pose,[.12,-.15,.08],atol=.01);assert rmse<.01;assert overlap>.99

def test_no_overlap_rejected():
    p=np.arange(300).reshape(-1,3).astype(float);_,rmse,overlap=icp_planar(p,cKDTree(p+1000),[0.,0.,0.]);assert np.isinf(rmse);assert overlap==0


def test_configurable_seaport_grid_extent_and_invalid_configuration():
    import pytest
    grid=Grid(.2,1200)
    assert grid.size == 1200
    assert grid.origin == pytest.approx(-120.0)
    assert grid.cells([[0.0,0.0],[119.8,119.8]]).tolist() == [[600,600],[1199,1199]]
    for resolution,size in [(0.0,600),(-0.1,600),(float('nan'),600),(.1,31),(.1,2049)]:
        with pytest.raises(ValueError):
            Grid(resolution,size)
