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


void check_collision(UAV_data A, UAV_data B, float safe_radius, float time_limit, bool check[][55]) 
{
    float dx = B.x - A.x;
    float dy = B.y - A.y;
    float dz = B.z - A.z;

    float dvx = B.vx - A.vx;
    float dvy = B.vy - A.vy;
    float dvz = B.vz - A.vz;

    float a = dvx*dvx + dvy*dvy + dvz*dvz;
    float b =  2.0f * (dx*dvx + dy*dvy + dz*dvz);
    float c = dx*dx + dy*dy + dz*dz - safe_radius*safe_radius;

    float delta = b*b - 4.0f*a*c;

    if (delta < 0) 
        return;

    if (delta == 0)
    {
        float t = -b / (2.0f * a);
        if (t >= 0 && t <= time_limit) 
        {
            cout << "   [!] WARNING: UAV " << A.id << " and UAV " << B.id 
                 << " is going to be COLLIDED in " << t << " s" << endl;
            check[A.id][B.id] = true;
        }
        return;
    }

    float sqrt_delta = sqrt(delta);
    float t1 = (-b - sqrt_delta) / (2.0f * a); 
    float t2 = (-b + sqrt_delta) / (2.0f * a);
    
    if (t1 >= 0 && t1 <= time_limit) 
    {
        cout << "   [!] WARNING: UAV " << A.id << " and UAV " << B.id 
                 << " is going to be COLLIDED in " << t1 << " s" << endl;
        check[A.id][B.id] = true;
    }

    else if (t1 < 0 && t2 > 0) 
    {
        cout << "   [X] DANGER: UAV " << A.id << " and UAV " << B.id 
             << " ARE VIOLATING SAFETY RANGE!" << endl;
        check[A.id][B.id] = true;
    }
    
    return;
}


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
            if (children[i]->contains(uav)) 
            {
                children[i]->insert(uav);
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
            // Nếu là nhánh, hỏi tiếp 8 đứa con
            for (int i = 0; i < 8; ++i) 
            {
                children[i]->query(target, range, found);
            }
        }
    }
};

bool input(vector<Raw_data>& input_data, const string& Input_file) 
{
    ifstream file(Input_file);
    if (!file.is_open()) 
    {
        return 0;
    }

    string line, token;
    getline(file, line); 

    while(getline(file, line))
    {
        stringstream ss(line);
        Raw_data uav;

        // Băm chuỗi (Bạn nhớ chỉnh lại thứ tự này cho khớp với file CSV bạn tự chế nhé)
        getline(ss, token, ','); uav.id = stoi(token);
        getline(ss, token, ','); uav.lat = stof(token);
        getline(ss, token, ','); uav.lon = stof(token);
        getline(ss, token, ','); uav.alt = stof(token);
        getline(ss, token, ','); uav.vel = stof(token);
        getline(ss, token, ','); uav.yaw = stof(token);
        getline(ss, token, ','); uav.pitch = stof(token);

        input_data.push_back(uav);
    }

    if (input_data.empty()) 
    {
        cout << "Warning: No data read from file " << Input_file << endl;
        return 0;
    }

    file.close();
    cout << "Finished reading " << input_data.size() << " lines of data." << endl;
    return 1;
    
}

bool normalizing(vector<Raw_data>& input_data, vector<UAV_data>& normalized_data) 
{
    float min_lat = input_data[0].lat, max_lat = input_data[0].lat;
    float min_lon = input_data[0].lon, max_lon = input_data[0].lon;
    float min_alt = input_data[0].alt, max_alt = input_data[0].alt;

    for (Raw_data i : input_data) 
    {
        min_lat = min(min_lat, i.lat);
        max_lat = max(max_lat, i.lat);
        min_lon = min(min_lon, i.lon);
        max_lon = max(min_lon, i.lon);
        min_alt = min(min_alt, i.alt);
        max_alt = max(min_alt, i.alt);
    }
    
    for (Raw_data uav : input_data) 
    {
        UAV_data p;
        p.id = uav.id;
        
        // Chuyển tọa độ về [-1, 1]
        p.x = 2.0f * ((uav.lon - min_lon) / (max_lon - min_lon)) - 1.0f; 
        p.y = 2.0f * ((uav.lat - min_lat) / (max_lat - min_lat)) - 1.0f;
        p.z = 2.0f * ((uav.alt - min_alt) / (max_alt - min_alt)) - 1.0f;

        // Tính toán vector vận tốc từ vận tốc, yaw và pitch
        float temp_vx = uav.vel * cos(uav.pitch * M_PI / 180.0f) * sin(uav.yaw * M_PI / 180.0f) ;
        float temp_vy =  uav.vel * cos(uav.pitch * M_PI / 180.0f) * cos(uav.yaw * M_PI / 180.0f);
        float temp_vz = uav.vel * sin(uav.pitch * M_PI / 180.0f);

        // Cực kỳ quan trọng: Vận tốc cũng phải bị Scale theo cùng tỷ lệ không gian
        p.vx = 2.0f * temp_vx / (max_lon - min_lon) - 1.0f;
        p.vy = 2.0f * temp_vy / (max_lat - min_lat) - 1.0f;
        p.vz = 2.0f * temp_vz / (max_alt - min_alt) - 1.0f;

        normalized_data.push_back(p);
    }
    cout << "Data normalization to [-1, 1] range completed." << endl;
    return 1;
}

bool buildTree(OctreeNode* root, vector<UAV_data>& normalized_data) 
{
    int inserted_count = 0;
    for (UAV_data i : normalized_data) 
    {
        if (root->insert(i)) 
        {
            inserted_count++;
        }
    }
    cout << "Successfully inserted " << inserted_count << " UAVs into Octree!" << endl;
    return 1;
}


int main ()
{
    vector <Raw_data> input_data;
    string Input_file = "input_data.csv";
    if(!input(input_data, Input_file))
    {
        cerr << "Error: Cannot open file " << Input_file << std::endl;
        return -1;
    }

    vector <UAV_data> normalized_data;
    if(!normalizing(input_data, normalized_data))
    {
        cerr << "Error: Data normalization failed." << std::endl;
        return -1;
    }


    Point3d root_center = {0.0f, 0.0f, 0.0f};
    int max_capacity = 5; 
    OctreeNode* root = new OctreeNode(root_center, 1.0f, max_capacity);
    if(!buildTree(root, normalized_data))
    {
        cerr << "Error: Building Octree failed." << std::endl;
        return -1;
    }

   
    cout << "\n---Collision warning system is loading---" << endl;
    
    float search_range = 0.2f;   
    float safe_radius = 0.05f;   
    float time_limit = 5.0f;    
    bool check[55][55] = {};

    for (UAV_data uav : normalized_data) 
    {
        vector <UAV_data> neighbors;
        
        root->query(uav, search_range, neighbors);

        for (UAV_data neighbor : neighbors) 
        {
            if (uav.id < neighbor.id && !check[uav.id][neighbor.id]) 
            {
                check_collision(uav, neighbor, safe_radius, time_limit, check);
            }
        }
    }

    cout << "Done!" << endl;

    delete root;

    return 0;
}
