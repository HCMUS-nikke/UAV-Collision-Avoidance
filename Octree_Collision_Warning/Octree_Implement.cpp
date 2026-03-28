#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath> 

using namespace std;

struct Raw_data 
{
    int id;
    float lat, lon, alt;
    float vel, yaw, pitch; 
};

struct Point3d
{
    float x, y, z; 
};

struct UAV_data
{
    int id;
    float x, y, z;
    float vx, vy, vz; 
};

class OctreeNode 
{
private:
    Point3d center;
    float half_size;
    int max_capacity;
    vector <UAV_data> uavs;
    bool is_leaf;
    OctreeNode* children[8];

    void subdivide()
    {
        float new_size = half_size / 2.0f;
        children[0] = new OctreeNode({center.x + new_size, center.y + new_size, center.z + new_size}, new_size, max_capacity);
        children[1] = new OctreeNode({center.x + new_size, center.y + new_size, center.z - new_size}, new_size, max_capacity);
        children[2] = new OctreeNode({center.x + new_size, center.y - new_size, center.z + new_size}, new_size, max_capacity);
        children[3] = new OctreeNode({center.x + new_size, center.y - new_size, center.z - new_size}, new_size, max_capacity);
        children[4] = new OctreeNode({center.x - new_size, center.y + new_size, center.z + new_size}, new_size, max_capacity);
        children[5] = new OctreeNode({center.x - new_size, center.y + new_size, center.z - new_size}, new_size, max_capacity);
        children[6] = new OctreeNode({center.x - new_size, center.y - new_size, center.z + new_size}, new_size, max_capacity);
        children[7] = new OctreeNode({center.x - new_size, center.y - new_size, center.z - new_size}, new_size, max_capacity);
        is_leaf = false;

        for (UAV_data i : uavs) 
        {
            for (int j = 0; j < 8; j++) 
            {
                if (children[j]->insert(i)) 
                {

                    break;
                }
            }
        }
        uavs.clear();
    }

public:
    OctreeNode(Point3d c, float h_size, int max_cap)
    {
        center = c;
        half_size = h_size;
        max_capacity = max_cap;
        is_leaf = true;
        for (int i = 0; i < 8; i++)
        {
            children[i] = nullptr;
        }
    }

    bool contains(UAV_data uav) 
    {
        return (uav.x >= center.x - half_size && uav.x < center.x + half_size &&
                uav.y >= center.y - half_size && uav.y < center.y + half_size &&
                uav.z >= center.z - half_size && uav.z < center.z + half_size);
    }
    
    bool insert(UAV_data uav) 
    {
        if (!contains(uav))
        {
            return false;
        }
        if (is_leaf) 
        {
            if (uavs.size() < max_capacity) 
            {
                uavs.push_back(uav);
                return true;
            } 
            else 
            {
                subdivide();
            }
        }
        for (int i = 0; i < 8; ++i) 
        {
            if (children[i]->insert(uav)) 
            {
                return true;
            }
                
        }
        return false;
    }

    ~OctreeNode() 
    {
        if (!is_leaf) 
        {
            for (int i = 0; i < 8; ++i) 
            {
                delete children[i];
            }
        }
    }

    
    void query(UAV_data target, float range, vector<UAV_data>& found) 
    {
        
        if (target.x + range < center.x - half_size || target.x - range > center.x + half_size ||
            target.y + range < center.y - half_size || target.y - range > center.y + half_size ||
            target.z + range < center.z - half_size || target.z - range > center.z + half_size) 
        {
            return;
        }

        if (is_leaf) 
        {
            
            for (UAV_data p : uavs) 
            {
                if (p.id != target.id) found.push_back(p);
            }
        } 
        else 
        {
            for (int i = 0; i < 8; ++i) 
            {
                children[i]->query(target, range, found);
            }
        }
    }
};
