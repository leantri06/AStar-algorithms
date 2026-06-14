# A* Pathfinding Algorithms - Discrete Structure (CO1007)

Một dự án triển khai thuật toán **A* (A-Star)** trong nhiều ngữ cảnh và không gian bài toán khác nhau, thuộc khuôn khổ Bài tập lớn môn **Cấu trúc rời rạc (CO1007)** - Học kỳ 2 (2025-2026), Đại học Bách Khoa TP.HCM (HCMUT).

## 📌 Tổng quan dự án

Dự án sử dụng ngôn ngữ **C++** để giải quyết 4 bài toán tối ưu hóa đường đi bằng thuật toán A* kết hợp với các hàm Heuristic ($h(n)$) khác nhau:
1. **Degrees of Separation in a Social Network**: Tìm khoảng cách và chuỗi liên kết ngắn nhất giữa 2 người trong mạng xã hội qua ma trận kề.
2. **Drone Delivery in 2D Space**: Tìm đường bay tối ưu cho Drone dựa trên 3 chế độ khoảng cách: Manhattan, Euclidean, và Chebyshev.
3. **Warehouse Robot Navigation**: Điều hướng robot trong kho hàng mã hóa dạng lưới $m \times n$ có chướng ngại vật (hỗ trợ di chuyển 8 hướng, tính phí di chuyển chéo).
4. **Evacuation Route Planning**: Tự động chuyển đổi sơ đồ mặt bằng tòa nhà thành ma trận trọng số và tìm lộ trình thoát hiểm an toàn nhất đến cửa ra.

---

## 🛠️ Yêu cầu môi trường

* **Ngôn ngữ**: C++11 hoặc mới hơn.
* **Thư viện sử dụng**: Chỉ sử dụng các thư viện chuẩn (STL): `<iostream>`, `<fstream>`, `<string>`, `<cmath>`, `<vector>`, `<algorithm>`.
* **Trình biên dịch**: `g++` hoặc các IDE hỗ trợ C++ chuẩn (VS Code, Code::Blocks, CLion).

---

## 📊 Cấu trúc dữ liệu cốt lõi

Tất cả các bài toán đều trả về một danh sách liên kết đơn `PathNode` chứa lộ trình từ điểm bắt đầu đến điểm đích:

```cpp
struct PathNode {
    std::string name; // Tên nút hoặc hướng di chuyển (Up, Down, Up-Left,...)
    double f;         // Tổng chi phí ước tính: f(n) = g(n) + h(n)
    double g;         // Chi phí thực tế từ điểm bắt đầu đến nút hiện tại
    double h;         // Chi phí ước tính từ nút hiện tại đến đích
    PathNode* next;   // Con trỏ đến nút tiếp theo trong lộ trình
};

```

---

## 🚀 Hướng dẫn cài đặt và chạy thử

### 1. Clone dự án về máy

```bash
git clone [https://github.com/your-username/your-repo-name.git](https://github.com/your-username/your-repo-name.git)
cd your-repo-name

```

### 2. Biên dịch dự án

Sử dụng dòng lệnh `g++` để biên dịch toàn bộ các file trong thư mục:

```bash
g++ -Iinclude main.cpp src/Algo.cpp src/PathNode.cpp -o astar_demo

```

### 3. Chạy chương trình

```bash
./astar_demo

```

---

## 🔍 Chi tiết các hàm triển khai

Các hàm được chuẩn hóa theo yêu cầu hệ thống Auto-testing của LMS:

* **Task 1**: Mạng xã hội

```cpp
  PathNode* findSocialPath(double adjMatrix[100][100], int startPerson, int goalPerson);

```

* **Task 2**: Drone giao hàng (Mode 1: Manhattan, Mode 2: Euclidean, Mode 3: Chebyshev)

```cpp
  PathNode* findDronePath(double weightMatrix[100][100], int coords[100][2], int startPoint, int goalPoint, int mode);

```

* **Task 3**: Robot kho hàng (Di chuyển ngang/dọc: `1.0`, di chuyển chéo: `1.5`)

```cpp
  PathNode* findWarehousePath(int warehouse[100][100], int m, int n, int startX, int startY, int goalX, int goalY, int mode);

```

* **Task 4**: Sơ đồ thoát hiểm (Tự động dựng ma trận trọng số đồ thị từ lưới mặt bằng)

```cpp
  PathNode* findEvacuationPath(int floorPlan[100][100], int m, int n, int startX, int startY, int exitX, int exitY, double weightMatrix[100][100], int mode);

```

---

## 📝 Báo cáo đính kèm

Chi tiết về cách tính toán Heuristic bằng giải thuật BFS (Task 1), các ví dụ chạy tay (Manual Runs) so sánh ưu/nhược điểm của từng hàm khoảng cách, và cơ chế chuyển đổi Grid sang Ma trận trọng số (Task 4) được trình bày đầy đủ trong file báo cáo dạng PDF.

---

## 🏫 Thông tin bản quyền & Môn học

* **Môn học**: Cấu trúc rời rạc (CO1007) - Học kỳ 2 / Năm học 2025-2026.
* **Trường**: Đại học Bách Khoa - ĐHQG TP.HCM (HCMUT).
* **Giảng viên phụ trách**: TS. Nguyễn An Khương, cô Lê Hồng Trang, thầy Trần Tuấn Anh, thầy Phạm An Vinh, thầy Mai Xuân Toàn, thầy Trần Hồng Tài.