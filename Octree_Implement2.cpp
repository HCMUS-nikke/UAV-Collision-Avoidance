#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

using namespace std;

// ==========================================
// Part 1: Data structure
// ==========================================
// The raw data pre-processing
struct RawUAV 
{
    int id;
    float lat, lon, alt;
};

// Data after processing 
struct Point 
{
    int uav_id;
    float x, y, z; 
};

// ==========================================
// Part 2: Octree implementation
// ==========================================
class OctreeNode 
{
private:
    Point center;               // The block center
    float half_size;            // Distance from center to surface
    int max_points;             // Max capacity
    vector<Point> points;       // List of UAVs in 1 block
    OctreeNode* children[8];    // Pointer to 8 child nodes
    bool is_leaf;               // Check if the node is a leaf or not

public:
    // Node init
    OctreeNode(Point c, float h_size, int max_p) 
    {
        center = c;
        half_size = h_size;
        max_points = max_p;
        is_leaf = true;
        for (int i = 0; i < 8; i++)
        {
            children[i] = nullptr;
        }
            
    }

    // Destructor to free memory (prevent memory leak)
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

    // Export Octree box coordinates to CSV file
    void exportBoxes(ofstream& file) 
    {
        // Write the center coordinates and half_size of the current block
        file << center.x << "," << center.y << "," << center.z << "," << half_size << "\n";
        
        // If it is not a leaf node, continue recursing down to child blocks
        if (!is_leaf) 
        {
            for (int i = 0; i < 8; i++) 
            {
                if (children[i] != nullptr) 
                {
                    children[i]->exportBoxes(file);
                }
            }
        }
    }

    // Check if the UAV is in the block
    bool contains(Point p) 
    {
        return (p.x >= center.x - half_size && p.x <= center.x + half_size &&
                p.y >= center.y - half_size && p.y <= center.y + half_size &&
                p.z >= center.z - half_size && p.z <= center.z + half_size);
    }

    // To divide a current block into 8 parts
    void subdivide() 
    {
        float quarter = half_size / 2.0f;
        int i = 0;
        // Iterate through 8 corners to create 8 child nodes
        for (int x = -1; x <= 1; x += 2) 
        {
            for (int y = -1; y <= 1; y += 2) 
            {
                for (int z = -1; z <= 1; z += 2) 
                {
                    Point child_center = {0, center.x + x * quarter, center.y + y * quarter, center.z + z * quarter};
                    children[i++] = new OctreeNode(child_center, quarter, max_points);
                }
            }
        }
        is_leaf = false;
        
        // Move current UAVs down to child nodes
        for (auto& p : points) 
        {
            for (int j = 0; j < 8; ++j) 
            {
                if (children[j]->insert(p)) break;
            }
        }
        points.clear(); // Parent node no longer contains UAVs directly
    }

    // Function to insert UAV into the tree
    bool insert(Point p) 
    {
        // If the point is not in this block, ignore it
        if (!contains(p)) return false;

        // If the block is not subdivided and still has capacity
        if (is_leaf && points.size() < max_points) 
        {
            points.push_back(p);
            return true;
        }

        // If the block is full, subdivide it (if not already)
        if (is_leaf) subdivide();

        // Try to insert into child nodes
        for (int i = 0; i < 8; ++i) 
        {
            if (children[i]->insert(p)) return true;
        }
        return false;
    }
};

// ==========================================
// PART 3: MAIN FUNCTION - EXECUTION
// ==========================================
int main() 
{
    vector<RawUAV> raw_data;
    string filename = "uav_data.csv";
    
    // ==========================================
    // STEP 1: READ CSV FILE
    // ==========================================
    ifstream file(filename);
    if (!file.is_open()) 
    {
        cerr << "Error: Cannot open file " << filename << std::endl;
        return 1;
    }

    string line;
    // Skip the first header line
    getline(file, line); 

    while (getline(file, line)) 
    {
        stringstream ss(line);
        string token;
        RawUAV uav;

        // Column 0: UAV_ID
        getline(ss, token, ',');
        uav.id = stoi(token);

        // Column 1: Timestamp (Skip and do not save)
        getline(ss, token, ',');

        // Column 2: Latitude
        getline(ss, token, ',');
        uav.lat = stof(token);

        // Column 3: Longitude
        getline(ss, token, ',');
        uav.lon = stof(token);

        // Column 4: Altitude
        getline(ss, token, ',');
        uav.alt = stof(token);

        // Add to list
        raw_data.push_back(uav);
    }
    file.close();
    cout << "Finished reading " << raw_data.size() << " lines of data." << endl;

    // ==========================================
    // STEP 2: NORMALIZE DATA (MIN-MAX SCALING)
    // ==========================================
    if (raw_data.empty()) return 0;

    // Find Min and Max values of the 3 axes
    float min_lat = raw_data[0].lat, max_lat = raw_data[0].lat;
    float min_lon = raw_data[0].lon, max_lon = raw_data[0].lon;
    float min_alt = raw_data[0].alt, max_alt = raw_data[0].alt;

    for (const auto& uav : raw_data) 
    {
        if (uav.lat < min_lat) min_lat = uav.lat;
        if (uav.lat > max_lat) max_lat = uav.lat;
        if (uav.lon < min_lon) min_lon = uav.lon;
        if (uav.lon > max_lon) max_lon = uav.lon;
        if (uav.alt < min_alt) min_alt = uav.alt;
        if (uav.alt > max_alt) max_alt = uav.alt;
    }

    vector<Point> normalized_data;
    for (const auto& uav : raw_data) 
    {
        Point p;
        p.uav_id = uav.id;
        // Apply formula (X - Min) / (Max - Min)
        p.x = (uav.lon - min_lon) / (max_lon - min_lon); // X-axis is usually Longitude
        p.y = (uav.lat - min_lat) / (max_lat - min_lat); // Y-axis is usually Latitude
        p.z = (uav.alt - min_alt) / (max_alt - min_alt); // Z-axis is Altitude
        normalized_data.push_back(p);
    }
    cout << "Data normalization to [0, 1] range completed." << endl;

    // ==========================================
    // STEP 3: INSERT INTO OCTREE
    // ==========================================
    Point root_center = {0, 0.5f, 0.5f, 0.5f}; // uav_id = 0, x = 0.5, y = 0.5, z = 0.5
    OctreeNode* root = new OctreeNode(root_center, 0.5f, 10); // Capacity of 10 UAVs/block
    
    int inserted_count = 0;
    for (const auto& p : normalized_data) 
    {
        if (root->insert(p)) 
        {
            inserted_count++;
        }
    }
    cout << "Successfully inserted " << inserted_count << " UAVs into Octree!" << endl;

    // ==========================================
    // STEP 4: EXPORT OCTREE DATA TO FILE
    // ==========================================
    ofstream box_file("octree_boxes.csv");
    if (box_file.is_open()) 
    {
        // Write header
        box_file << "x,y,z,half_size\n";
        
        // Start recursing from root to write all nodes
        root->exportBoxes(box_file);
        
        box_file.close();
        cout << "Successfully exported octree_boxes.csv for 3D visualization." << endl;
    } 
    else 
    {
        cerr << "Error: Cannot create file octree_boxes.csv" << endl;
    }

    // Clean up memory before exiting the program
    delete root;

    return 0;
}
