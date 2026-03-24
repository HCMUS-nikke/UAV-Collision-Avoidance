#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
using namespace std;

// ==========================================
// PHẦN 1: CẤU TRÚC DỮ LIỆU
// ==========================================
// Cấu trúc lưu dữ liệu gốc chưa chuẩn hóa
struct RawUAV 
{
    int id;
    float lat, lon, alt;
};

// Cấu trúc lưu dữ liệu đã chuẩn hóa (dùng cho Octree)
struct Point 
{
    int uav_id;
    float x, y, z; 
};

// ==========================================
// PHẦN 2: CÀI ĐẶT THUẬT TOÁN OCTREE
// ==========================================
class OctreeNode 
{
private:
    Point center;               // Tâm của khối không gian
    float half_size;            // Nửa chiều dài cạnh của khối (khoảng cách từ tâm đến mặt)
    int max_points;             // Số UAV tối đa trong 1 khối trước khi bị chia nhỏ
    vector<Point> points;  // Danh sách UAV trong node này
    OctreeNode* children[8];    // Con trỏ tới 8 khối con (8 Octants)
    bool is_leaf;               // Đánh dấu xem có phải node lá (chưa bị chia) không

public:
    // Khởi tạo Node
    OctreeNode(Point c, float h_size, int max_p) 
    {
        center = c;
        half_size = h_size;
        max_points = max_p;
        is_leaf = true;
        for (int i = 0; i < 8; ++i) children[i] = nullptr;
    }

    // Hủy Node để giải phóng bộ nhớ (tránh memory leak)
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

    // Hàm kiểm tra xem 1 điểm có nằm trong khối này không
    bool contains(Point p) 
    {
        return (p.x >= center.x - half_size && p.x <= center.x + half_size &&
                p.y >= center.y - half_size && p.y <= center.y + half_size &&
                p.z >= center.z - half_size && p.z <= center.z + half_size);
    }

    // Hàm chia khối hiện tại thành 8 khối con
    void subdivide() 
    {
        float quarter = half_size / 2.0f;
        int i = 0;
        // Duyệt qua 8 góc để tạo 8 khối con
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
        
        // Đẩy các UAV hiện tại xuống các node con
        for (auto& p : points) 
        {
            for (int j = 0; j < 8; ++j) 
            {
                if (children[j]->insert(p)) break;
            }
        }
        points.clear(); // Node cha không chứa UAV nữa
    }

    // Hàm chèn UAV vào cây
    bool insert(Point p) 
    {
        // Nếu điểm không nằm trong khối này thì bỏ qua
        if (!contains(p)) return false;

        // Nếu khối chưa bị chia và vẫn còn sức chứa
        if (is_leaf && points.size() < max_points) 
        {
            points.push_back(p);
            return true;
        }

        // Nếu khối đã đầy thì tiến hành chia nhỏ (nếu chưa chia)
        if (is_leaf) subdivide();

        // Thử chèn vào các node con
        for (int i = 0; i < 8; ++i) 
        {
            if (children[i]->insert(p)) return true;
        }
        return false;
    }
};

// ==========================================
// PHẦN 3: HÀM CHÍNH (MAIN) - THỰC THI
// ==========================================
int main() 
{
    vector<RawUAV> raw_data;
    string filename = "uav_data.csv";
    
    // ==========================================
    // BƯỚC 1: ĐỌC FILE CSV
    // ==========================================
    ifstream file(filename);
    if (!file.is_open()) 
    {
        cerr << "Loi: Khong the mo file " << filename << std::endl;
        return 1;
    }

    string line;
    // Bỏ qua dòng tiêu đề (header) đầu tiên
    getline(file, line); 

    while (getline(file, line)) 
    {
        stringstream ss(line);
        string token;
        RawUAV uav;

        // Cột 0: UAV_ID
        getline(ss, token, ',');
        uav.id = stoi(token);

        // Cột 1: Timestamp (Bỏ qua không lưu)
        getline(ss, token, ',');

        // Cột 2: Latitude
        getline(ss, token, ',');
        uav.lat = stof(token);

        // Cột 3: Longitude
        getline(ss, token, ',');
        uav.lon = stof(token);

        // Cột 4: Altitude
        getline(ss, token, ',');
        uav.alt = stof(token);

        // Thêm vào danh sách
        raw_data.push_back(uav);
    }
    file.close();
    cout << "Da doc xong " << raw_data.size() << " dong du lieu." << endl;

    // ==========================================
    // BƯỚC 2: CHUẨN HÓA DỮ LIỆU (MIN-MAX SCALING)
    // ==========================================
    if (raw_data.empty()) return 0;

    // Tìm giá trị Min, Max của 3 trục
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
        // Áp dụng công thức (X - Min) / (Max - Min)
        p.x = (uav.lon - min_lon) / (max_lon - min_lon); // Trục X thường là Longitude
        p.y = (uav.lat - min_lat) / (max_lat - min_lat); // Trục Y thường là Latitude
        p.z = (uav.alt - min_alt) / (max_alt - min_alt); // Trục Z là Altitude
        normalized_data.push_back(p);
    }
    cout << "Da chuan hoa xong du lieu ve khoang [0, 1]." << endl;

    // ==========================================
    // BƯỚC 3: ĐƯA VÀO OCTREE (Dùng lại code class OctreeNode ở trên)
    // ==========================================
    Point root_center = {0, 0.5f, 0.5f, 0.5f}; // uav_id = 0, x = 0.5, y = 0.5, z = 0.5
    OctreeNode* root = new OctreeNode(root_center, 0.5f, 10); // Sức chứa 10 UAV/khối
    
    int inserted_count = 0;
    for (const auto& p : normalized_data) 
    {
        if (root->insert(p)) 
        {
            inserted_count++;
        }
    }
    cout << "Da chen thanh cong " << inserted_count << " UAV vao Octree!" << endl;

    // Dọn dẹp bộ nhớ trước khi thoát chương trình
    delete root;

    return 0;
}