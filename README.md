# Delta 4DOF

## Cách mở chương trình

Project này dùng ROS 2 Jazzy và Gazebo để chạy mô phỏng robot Delta 4DOF.

---

## Lần đầu tiên sau khi tải project về

Sau khi tải project từ GitHub về, mở thư mục:

delta_4dof

Sau đó bấm file:

RUN_DELTA_4DOF.sh

Chọn:

Run as a Program

File RUN_DELTA_4DOF.sh sẽ tự động:

1. Kiểm tra các package cần thiết
2. Build workspace nếu chưa build
3. Tạo shortcut ngoài Desktop
4. Mở chương trình Delta 4DOF

---

## Các lần sau

Sau lần chạy đầu tiên, chương trình sẽ tạo shortcut ngoài Desktop tên là:

Delta 4DOF Control

Từ lần thứ hai trở đi, chỉ cần bấm shortcut này trên Desktop để mở chương trình.

Shortcut này sẽ gọi file:

run_delta_control.sh

để chạy:

Gazebo + bridge + feedback + giao diện điều khiển

---

## Nếu không bấm được bằng chuột

Mở terminal trong thư mục delta_4dof, sau đó chạy:

chmod +x RUN_DELTA_4DOF.sh
./RUN_DELTA_4DOF.sh

Nếu muốn build lại workspace từ đầu:

./RUN_DELTA_4DOF.sh --rebuild

---

## Tóm tắt

Lần đầu:

RUN_DELTA_4DOF.sh trong thư mục delta_4dof

Các lần sau:

Delta 4DOF Control ngoài Desktop
