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


def test_fresh_lidar_ray_clears_stale_occupied_hit_but_keeps_new_endpoint():
    """An obstacle removed between scans must not remain as a phantom Nav2 wall."""
    g = Grid(.1, 120)
    origin = [0., 0., .5]
    g.update(np.array([[2., 0., .5]]), origin, 0.)
    old_cell = g.cells([[2., 0.]])[0]
    assert g.data()[old_cell[1], old_cell[0]] == 100

    # New ray reaches a more distant obstacle in the same direction:
    # the earlier 2m endpoint is now a directly observed FREE cell.
    g.update(np.array([[4., 0., .5]]), origin, 0.)
    new_cell = g.cells([[4., 0.]])[0]
    assert g.data()[old_cell[1], old_cell[0]] == 0
    assert g.data()[new_cell[1], new_cell[0]] == 100


def test_fresh_endpoints_win_even_if_other_ray_passes_through_them():
    g = Grid(.1, 120)
    origin = [0., 0., .5]
    g.update(np.array([[2., 0., .5], [4., 0., .5]]), origin, 0.)
    close_cell = g.cells([[2., 0.]])[0]
    far_cell = g.cells([[4., 0.]])[0]
    assert g.data()[close_cell[1], close_cell[0]] == 100
    assert g.data()[far_cell[1], far_cell[0]] == 100
