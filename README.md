# Moto Intersection — C++ / OpenGL

## Chạy
- Mở `run.bat` hoặc `build/TrafficSim.exe`.
- Mở `xem_xe.bat` để xem ngay hai mẫu xe ở cự ly gần.
- Mở `giao_thong.bat` để xem ngã tư đã có xe và hàng đợi (khởi tạo trước 90 giây mô phỏng).
- Biên dịch lại khi sửa mã: `build.bat` (cần g++ có trong PATH).
- Kiểm tra mô phỏng: `build\TrafficSim.exe --self-test`.
- Kiểm tra hiển thị tự động: `build\TrafficSim.exe --smoke`.

Dự án dùng GLFW có sẵn ở thư mục cha, không cần tải thêm tài nguyên.
File EXE liên kết tĩnh các runtime MinGW; cần driver hỗ trợ OpenGL 2.1 / GLSL 1.20.

## Phím
| Phím | Chức năng |
|---|---|
| 1 / 2 / 3 / 4 | Toàn cảnh / trên xuống / xem xe / theo xe được chọn |
| M (ở chế độ 3) | Đổi giữa Wave Alpha 110 và Vision tiêu chuẩn |
| O | Chuyển phép chiếu phối cảnh / trực giao; tên phép chiếu hiện trên HUD |
| G (ở chế độ 3) | Đường đo dài toàn xe (xanh), trục bánh (vàng), cao yên (đỏ) |
| Mũi tên | Xoay camera |
| Cuộn chuột | Zoom toàn cảnh hoặc cận cảnh mẫu xe |
| Space / dấu chấm | Tạm dừng / chạy một bước 1/60 giây khi đang dừng |
| R | Đặt lại; seed ngẫu nhiên cố định để tái hiện |
| W / N / C | Lưới / pháp tuyến / các mốc tạo biên dạng yên trong chế độ 3 |
| P / H | Quỹ đạo và vùng xung đột / chuyển Phong–Gouraud |
| Tab | Chọn xe tiếp theo theo ID |
| + / - | Tăng / giảm lưu lượng yêu cầu sinh xe |
| Esc | Thoát |

HUD dùng ASCII để tương thích font bitmap đang có trong bài tập.
Lưu lượng là số lần thử sinh xe mỗi giây; đầu đường bị đầy sẽ bỏ lượt sinh.

## Mã nguồn
- `Math.h`: vector, phép quay và de Casteljau.
- `BikeModel.*` dựng hai mẫu xe, vật liệu và các cụm chuyển động. `Geometry.*` vẽ lưới tam giác và hình học của cảnh.
- `Simulation.*`: bảng độ dài cung, đèn, hàng đợi, quyền vào ngã tư và khoảng cách an toàn.
- `Renderer.*`, `Shader.h`: camera, cảnh, HUD, GLSL Phong và OpenGL Gouraud.
- `main.cpp`: điều khiển, bước thời gian cố định và kiểm tra tự động.
- `Font8x8.*`: font public-domain có sẵn trong bài tập trước; giữ thông tin tác giả.

## Hai mẫu thực tế
Dùng catalogue Honda Việt Nam tháng 10/2024 cho [Wave Alpha 110](https://cdn.honda.com.vn/motorbikes-version-new/October2024/GSVYx3VwY3SVnUGeML2l.pdf) và tháng 11/2024 cho [Vision tiêu chuẩn](https://cdn.honda.com.vn/motorbikes-version-new/November2024/kZorUZNqFZXzhoNvPrsl.pdf). Tỷ lệ 1 đơn vị = 1 m.

| Mẫu | Dài × rộng × cao hãng (mm) | Trục bánh / cao yên (mm) | Lốp trước / sau |
|---|---|---|---|
| Wave Alpha 110 | 1.913 × 689 × 1.076 | 1.224 / 770 | 70/90-17 / 80/90-17 |
| Vision tiêu chuẩn | 1.871 × 686 × 1.101 | 1.255 / 761 | 80/90-14 / 90/90-14 |

C++ dựng các tiết diện theo ảnh rồi nội suy Hermite có giới hạn độ dốc để tạo yên và ốp thân. Đường Bézier bậc hai bo các góc viền đèn và gương. Chương trình lấy mẫu các bề mặt thành tam giác; tem hông được chia nhỏ rồi ánh xạ lên đúng bề mặt cong. Bánh quay theo quãng đường chia cho bán kính riêng của lốp trước hoặc sau. Mã giữ ma trận riêng cho thân, cụm lái và hai bánh. Xe cách camera trên 12 m dùng lưới ít tam giác hơn.

Chạy `build/TrafficSim.exe --model-test` để đo lưới ở tư thế thẳng lái. Phép kiểm tra đối chiếu chiều dài, rộng, khoảng cách hai trục và cao độ bề mặt yên tại vị trí ngồi. Bán kính lốp tính từ cỡ lốp danh nghĩa. Trong mô hình, cao độ đỉnh ốp tay lái lấy theo số cao trong catalogue. Kích thước gương, kích thước từng chi tiết, màu và đường cong ốp là ước lượng từ ảnh. Chiều cao lưới có gương của Wave khoảng 1.261 mm, Vision khoảng 1.286 mm. Chưa có CAD hoặc số đo xe thật để xác nhận sai số từng bề mặt.

Phím **3** mở cận cảnh, **M** đổi mẫu. Kết quả chụp từ chương trình nằm ở [ảnh hai mẫu](previews/Wave-Vision.png).
Chạy `build/TrafficSim.exe --model-preview` để xuất ảnh PPM ở góc ngang, trước, sau, chính diện và góc đo trực giao vào thư mục `build`. Ảnh được đọc trực tiếp từ framebuffer OpenGL.
Tiện ích tùy chọn `python tools/make_previews.py` ghép ảnh PNG (cần Pillow); chương trình C++ không cần Python/Pillow để chạy.

## Giao thông có trật tự

- Mỗi hướng có một hàng xe chạy trên quỹ đạo xác định, không vượt hàng. Xe dừng với đầu bánh trước còn nằm sau vạch dừng; vạch dừng đặt trước phần sang đường.
- Máy trạng thái: Bắc–Nam xanh 18 s → vàng 3 s → đỏ toàn bộ ít nhất 2 s → Đông–Tây xanh 18 s → vàng 3 s → đỏ toàn bộ ít nhất 2 s. Pha đỏ toàn bộ được kéo dài nếu xe của pha cũ chưa thoát hết; đèn cho hai trục không xanh đồng thời.
- Khi vàng, xe có đủ khoảng cách sẽ phanh dừng. Xe đã được phép đi và quá gần để dừng êm sẽ tiếp tục qua. Khoảng dừng ước lượng: `0.45*v + v*v/(2*2.5)` (m). Không cấp lượt mới trong pha vàng và không cho xe mới vào ở pha đỏ.
- Xe đã vào tiếp tục thoát ngã tư khi tín hiệu đổi màu. Xe đi thẳng ở hai hướng đối diện có thể qua đồng thời. Rẽ trái phải chờ nếu đang có quỹ đạo cắt ngang được sử dụng; phân lượt theo thời điểm tới vùng chờ, hòa thì theo ID.
- Ma trận xung đột được tính trước từ các quỹ đạo lấy mẫu; đoàn xe chung hướng được điều khiển bằng khoảng cách. Chỉ giữ quyền trong vùng giao cắt và một đoạn thoát, không khóa toàn bộ ngã tư cho từng xe.
- HUD hiển thị màu đèn của hai trục, thời gian còn lại của pha và độ dài hàng chờ. `0.0s` trong pha đỏ toàn bộ nghĩa là đang chờ xe thoát, không phải lỗi đếm giờ.

`--self-test` kiểm tra riêng dừng đỏ, hai trường hợp vàng, kéo dài đỏ toàn bộ, luồng đồng thời và nhường khi rẽ, sau đó chạy ba mật độ trong 30 phút mô phỏng mỗi mật độ. `--traffic-preview` xuất ảnh sáu pha và một chuỗi chuyển pha trực tiếp từ OpenGL.
Tiện ích tùy chọn `python tools/make_traffic_previews.py` (Pillow) xuất ảnh PNG và `previews/Traffic-flow.gif`; GIF phát nhanh gấp 2 lần thời gian mô phỏng. Khi chạy ứng dụng bình thường, bước mô phỏng là 1/60 giây.

## Giới hạn mô phỏng

Đây là mô hình vi mô trên các quỹ đạo cố định và điều phối xung đột bảo thủ. Chưa mô phỏng luồn lách, đổi làn, người đi bộ hoặc hiệu chỉnh hành vi bằng số liệu giao thông thật. Thông số đèn và phản ứng là lựa chọn của bài mô phỏng, không phải cấu hình thiết kế một nút giao ngoài thực tế. Khi đầu đường đầy, lượt sinh xe bị bỏ.

Chuyển động thường có giới hạn gia tốc/phanh; chặn khoảng cách cuối cùng là cơ chế an toàn và có thể dừng gấp. Hai mẫu xe dài lần lượt 1,913 m và 1,871 m. Cơ chế an toàn giữ khoảng cách tâm tối thiểu 2,25 m.

OpenGL thực hiện depth test và face culling. Nhóm tự sinh lưới, tính pháp tuyến, lập trình tô sáng Phong và viết logic mô phỏng; không tự viết bộ rasterizer/Z-buffer phần mềm.

Kết quả kiểm tra tại máy này nằm trong `build/test-results.txt` và `build/render-results.txt`.

## Ma trận tự cài đặt
- `Matrix.h`: `Matrix4x4`, `multipleMatrix`, `translationMatrix`, `scalingMatrix`, `rotationXMatrix`, `rotationYMatrix`, `rotationZMatrix`, `rotationAroundPointMatrix`, `transformPoint`, `lookAtMatrix`, `perspectiveMatrix`, `orthographicMatrix`.
- Quy ước vector cột: p' = M * p; T * R * S áp dụng S trước, R rồi T. Các hàm quay nhận góc theo độ.
- `Transform.h`: lưu phân cấp ma trận mô hình trên CPU, nhân View * Model bằng `multipleMatrix`, nạp kết quả bằng `glLoadMatrixd`. Không sử dụng glTranslate/glRotate/glScale/glMultMatrix để ghép các phép biến đổi.
- Cảnh 3D mặc định dùng phép chiếu phối cảnh. Phím O đổi sang trực giao (một dạng chiếu song song); khi đổi, giữ cùng tỷ lệ tại mặt phẳng qua tâm quan sát. HUD luôn dùng phép chiếu trực giao.
- Kiểm tra riêng: `build\TrafficSim.exe --matrix-test`; kiểm tra này cũng được gọi ở đầu `--self-test`.
