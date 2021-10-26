#include "../include/cpp_lane_line_detection.h"

using std::placeholders::_1;

LaneLineDetection::LaneLineDetection()
    : Node("cpp_lane_line_detection")
{

  loadParam();
  initNode();
  get_M_Minv();
}

void LaneLineDetection::loadParam()
{
  this->declare_parameter<string>("image_sub_topic_name", "/camera/image_raw");
  this->declare_parameter<string>("res_dir", "res_dir");
  this->declare_parameter<vector<double>>("camera_matrix", {});
  this->declare_parameter<vector<double>>("dist_coeff", {});

  vector<double> c_m;
  vector<double> d_c;

  this->get_parameter("image_sub_topic_name", image_sub_topic_name);
  this->get_parameter("res_dir", res_dir);
  this->get_parameter("camera_matrix", c_m);
  this->get_parameter("dist_coeff", d_c);

  // 另一种方法
  // this->declare_parameter("dist_coeff"，std::vector<double>{});
  // rclcpp::Parameter double_array_param = this->get_parameter("dist_coeff");
  // std::vector<double> d_c = double_array_param.as_double_array();

  camera_matirx = (Mat_<double>(3, 3) << c_m[0], c_m[1], c_m[2],
                   c_m[3], c_m[4], c_m[5],
                   c_m[6], c_m[7], c_m[8]);                                     // 相机内参
  dist_coeffs = (Mat_<double>(1, 5) << d_c[0], d_c[1], d_c[2], d_c[3], d_c[4]); //畸变系数

  left_curve_img = imread(res_dir.append("left_turn.png"));
  right_curve_img = imread(res_dir.append("right_turn.png"));
  keep_straight_img = imread(res_dir.append("straight.png"));
  //值归一化
  normalize(left_curve_img, left_curve_img, 0, 255, NORM_MINMAX, CV_8U);
  normalize(right_curve_img, right_curve_img, 0, 255, NORM_MINMAX, CV_8U);
  normalize(keep_straight_img, keep_straight_img, 0, 255, NORM_MINMAX, CV_8U);
}

void LaneLineDetection::initNode()
{
  processed_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
      "/image/lines", 3);

  image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      image_sub_topic_name, 3, std::bind(&LaneLineDetection::imageCallback, this, _1));
}

void LaneLineDetection::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{

  cv_bridge::CvImagePtr cv_ptr_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  // cvtColor(cv_ptr_img->image, cv_ptr_img->image, COLOR_BGR2RGB);
  Mat out_img;
  detectionForward(cv_ptr_img->image, out_img);

  sensor_msgs::msg::Image img_msg;
  cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", out_img).toImageMsg(img_msg);
  img_msg.header.frame_id = "camera";
  processed_image_pub->publish(img_msg);
}

void LaneLineDetection::detectionForward(const Mat &src, Mat &dst)
{
  Mat un_img, tr_img, th_img, draw_img;
  vector<Point> lp, rp;              //车道线坐标点
  int rightx_current, leftx_current; // frameNum; //车道线基坐标点x轴
  double distance_from_center, curvature;

  undistort(src, un_img, camera_matirx, dist_coeffs);
  warpPerspective(un_img, tr_img, M, src.size(), INTER_LINEAR);
  thresholdForword(tr_img, th_img);
  findLine(th_img, lp, rp, rightx_current, leftx_current, distance_from_center, curvature);
  drawArea(un_img, dst, lp, rp, Minv, distance_from_center, curvature);
  // dst = un_img;
  // imshow("result", th_img);
  // waitKey(5);
}

void LaneLineDetection::get_M_Minv()
{
  // const vector<Point2f> &src;
  // const vector<Point2f> &dst;

  //变形基础点
  std::vector<Point2f> src = {Point2f(203, 720),
                              Point2f(585, 460),
                              Point2f(695, 460),
                              Point2f(1127, 720)};
  std::vector<Point2f> dst = {Point2f(320, 720),
                              Point2f(320, 0),
                              Point2f(960, 0),
                              Point2f(960, 720)};

  // std::vector<Point2f> src = {Point2f(150, 720),
  //                                 Point2f(550, 460),
  //                                 Point2f(770, 460),
  //                                 Point2f(1200, 720)};
  // std::vector<Point2f> dst = {Point2f(100, 720),
  //                                 Point2f(100, 0),
  //                                 Point2f(1100, 0),
  //                                 Point2f(1100, 720)};

  M = getPerspectiveTransform(src, dst);
  Minv = getPerspectiveTransform(dst, src);
}

void LaneLineDetection::labSelect(const Mat &src, Mat &dst, const char &channel,
                                  const int &thresh_min, const int &thresh_max)
{
  Mat lab, grad;
  std::vector<Mat> channels;
  cvtColor(src, lab, COLOR_RGB2Lab);
  split(lab, channels);

  switch (channel)
  {
  case 'l':
    grad = channels.at(0);
    break;
  case 'a':
    grad = channels.at(1);
    break;
  case 'b':
    grad = channels.at(2);
    break;
  }

  double minv = 0.0, maxv = 0.0;
  minMaxIdx(grad, &minv, &maxv);
  //minMaxLoc(lab_b, &minv, &maxv, 0, 0);
  double th = 255 / maxv;

  if (maxv > 100.0)
    grad = grad * (255 / maxv); // 归一化

  //将阈值应用于b通道
  dst = Mat::zeros(grad.rows, grad.cols, CV_8UC1);

  if (th <= 1.5)
  {
    for (int i = 0; i < grad.rows; i++)
    {
      for (int j = 0; j < grad.cols; j++)
      {
        if (grad.at<uchar>(i, j) > thresh_min &&
            grad.at<uchar>(i, j) <= thresh_max)
        {
          dst.at<uchar>(i, j) = 255;
        }
      }
    }
  }

  // inRange(grad, thresh_min, thresh_max, dst);
  // threshold(grad,dst,thresh_min,thresh_max,THRESH_BINARY);
}

void LaneLineDetection::hlsSelect(const Mat &src, Mat &dst, const char &channel,
                                  const int &thresh_min, const int &thresh_max)
{
  Mat hls, grad;
  std::vector<Mat> channels;
  cvtColor(src, hls, COLOR_RGB2HLS);
  split(hls, channels);

  switch (channel)
  {
  case 'h':
    grad = channels.at(0);
    break;
  case 'l':
    grad = channels.at(1);
    break;
  case 's':
    grad = channels.at(2);
    break;
  }

  double minv = 0.0, maxv = 0.0;
  minMaxIdx(grad, &minv, &maxv);
  grad = grad * (255 / maxv); // 归一化

  //将阈值应用于b通道
  dst = Mat::zeros(grad.rows, grad.cols, CV_8UC1);

  for (int i = 0; i < grad.rows; i++)
  {
    for (int j = 0; j < grad.cols; j++)
    {
      if (grad.at<uchar>(i, j) > thresh_min &&
          grad.at<uchar>(i, j) <= thresh_max)
      {
        dst.at<uchar>(i, j) = 255;
      }
    }
  }

  // inRange(grad, thresh_min, thresh_max, dst);
  // threshold(grad,dst,thresh_min,thresh_max,THRESH_BINARY);
}

void LaneLineDetection::thresholdForword(Mat src, Mat &dst)
{

  Mat hls, lab;
  cvtColor(src, src, COLOR_BGR2RGB);
  labSelect(src, lab, 'b', 195, 255);
  hlsSelect(src, hls, 'l', 200, 255);

  dst = hls | lab;
}

Mat LaneLineDetection::polyfit(vector<Point> &in_point, int n)
{
  int size = in_point.size();
  int x_num = n + 1; //构造矩阵U和Y
  Mat mat_u(size, x_num, CV_64F);
  Mat mat_y(size, 1, CV_64F);
  for (int i = 0; i < mat_u.rows; ++i)
  {
    for (int j = 0; j < mat_u.cols; ++j)
    {
      mat_u.at<double>(i, j) = pow(in_point[i].y, j); //in_point[i].y为以y为递增坐标
    }
  }
  for (int i = 0; i < mat_y.rows; ++i)
  {
    mat_y.at<double>(i, 0) = in_point[i].x;
  } //矩阵运算，获得系数矩阵K
  Mat mat_k(x_num, 1, CV_64F);
  mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
  // cout << mat_k << endl;
  return mat_k;
}

vector<Point> LaneLineDetection::polyval(const Mat &mat_k,
                                         const vector<Point> &src, int n)
{
  vector<Point> ip;
  // cout<<src.back().y<<"kkk"<<src.front().y<<endl;
  for (int i = src.back().y; i < src.front().y; i++)
  { //从y=0开始计算，分别计算出x的值
    Point ipt;
    ipt.x = 0;
    ipt.y = i;
    for (int j = 0; j < n + 1; j++)
    {
      ipt.x += mat_k.at<double>(j, 0) * pow(i, j); //NOTE多项式计算
    }
    ip.push_back(ipt);
  }
  return ip;
}

void LaneLineDetection::findLine(const Mat &src, vector<Point> &lp,
                                 vector<Point> &rp, int &rightx_current,
                                 int &leftx_current, double &vehicle_offset,
                                 double &curvature)
{
  Mat hist, nonzero, l, r;
  vector<Point> nonzerol, nonzeror, lpoint, rpoint;
  int midpoint;

  Point leftx_base, rightx_base;
  //选择滑窗个数
  int nwindows = 9;
  //设置窗口高度
  int window_height = int(src.rows / nwindows);
  //设置窗口宽度
  int margin = 100;
  //设置非零像素坐标最少个数
  unsigned int minpix = 50;
  /*TODO 加入if设置图像连续性，如果leftx_current和rightx_current为零，
   则认为第一次执行，需要计算该两点，如果已经计算了，则不许再次计算。*/
  //rowrange图像区域分割
  //将图像处理为一行，以行相加为方法
  reduce(src.rowRange(src.rows / 2, src.rows), hist, 0, REDUCE_SUM, CV_32S);
  midpoint = int(hist.cols / 2); //中分
  //将hist分为左右分别储存，并找出最大值
  //minMaxIdx针对多通道，minMaxLoc针对单通道
  minMaxLoc(hist.colRange(0, midpoint), NULL, NULL, NULL, &leftx_base);
  minMaxLoc(hist.colRange(midpoint, hist.cols), NULL, NULL, NULL, &rightx_base);
  //左右车道线基础点
  rightx_current = rightx_base.x + midpoint;

  //提前存入该基础点坐标
  lpoint.push_back(Point(leftx_current, src.rows));
  rpoint.push_back(Point(rightx_current, src.rows));
  // lpoint.push_back(Point(leftx_current, y_current));
  // rpoint.push_back(Point(rightx_current, y_current));
  Mat show_point = src.clone(); //Mat::zeros(src.rows, src.cols, CV_8UC3);
  cvtColor(show_point, show_point, CV_GRAY2BGR);
  for (int i = 0; i < nwindows; i++)
  {
    int win_y_low = src.rows - (i + 1) * window_height;
    // y_current -= window_height;
    // int win_y_low = y_current;
    //计算选框x坐标点，并将计算结果限制在图像坐标内
    int win_xleft_low = leftx_current - margin;
    win_xleft_low = win_xleft_low > 0 ? win_xleft_low : 0;
    win_xleft_low = win_xleft_low < src.cols / 2 ? win_xleft_low : src.cols / 2;
    //int win_xleft_high = leftx_current + margin;
    int win_xright_low = rightx_current - margin;
    win_xright_low = win_xright_low > 0 ? win_xright_low : 0;
    win_xright_low = win_xright_low < src.cols ? win_xright_low : src.cols;
    // RCLCPP_INFO(this->get_logger(), "win_xright_low %d", win_xright_low);
    // RCLCPP_INFO(this->get_logger(), "win_xleft_low %d", win_xleft_low);
    //int win_xright_high = rightx_current + margin;
    //NOTE要确保参数都大于0，且在src图像范围内，不然会报错
    int bias_right = 2 * margin;
    bias_right = win_xright_low + bias_right < src.cols ? bias_right : src.cols - win_xright_low;
    l = src(Rect(win_xleft_low, win_y_low, 2 * margin, window_height));
    r = src(Rect(win_xright_low, win_y_low, bias_right, window_height));
    //NOTE 把像素值不为零的像素坐标存入矩阵
    findNonZero(l, nonzerol);
    findNonZero(r, nonzeror);

    //计算每个选框的leftx_current和rightx_current中心点
    if (nonzerol.size() > minpix)
    {
      int leftx = 0;
      for (auto &n : nonzerol)
      {
        leftx += n.x;
      }
      leftx_current = win_xleft_low + leftx / nonzerol.size();
    }
    if (nonzeror.size() > minpix)
    {
      int rightx = 0;
      for (auto &n : nonzeror)
      {
        rightx += n.x;
      }
      rightx_current = win_xright_low + rightx / nonzeror.size();
    }
    rectangle(show_point, Rect(win_xleft_low, win_y_low, 2 * margin, window_height), Scalar(0, 0, 255), 1, LINE_8, 0);
    rectangle(show_point, Rect(win_xright_low, win_y_low, 2 * margin, window_height), Scalar(0, 255, 0), 1, LINE_8, 0);
    //将中心点坐标存入容器
    lpoint.push_back(Point(leftx_current, win_y_low));
    rpoint.push_back(Point(rightx_current, win_y_low));
    circle(show_point, Point(leftx_current, win_y_low), 3, Scalar(0, 0, 255));
    circle(show_point, Point(rightx_current, win_y_low), 3, Scalar(0, 255, 0));
  }
  //拟合左右车道线坐标
  Mat leftx = polyfit(lpoint, 2);
  Mat rightx = polyfit(rpoint, 2);
  //计算拟合曲线坐标
  lp = polyval(leftx, lpoint, 2);
  rp = polyval(rightx, rpoint, 2);

  double ym = 30 / 720;
  double xm = 3.7 / 700;
  double y_eval = 719 * ym;

  double left_curveR =
      pow(1 + pow(2 * leftx.at<double>(0, 2) * y_eval + leftx.at<double>(0, 1), 2), 1.5) /
      fabs(2 * leftx.at<double>(0, 2));
  double right_curveR =
      pow(1 + pow(2 * rightx.at<double>(0, 2) * y_eval + rightx.at<double>(0, 1), 2), 1.5) /
      fabs(2 * rightx.at<double>(0, 2));
  // RCLCPP_INFO(this->get_logger(), "curve %f", std::min(left_curveR, right_curveR));
  curvature = std::min(left_curveR, right_curveR);

  //计算车道偏离距离
  int lane_width = abs(rpoint.front().x - lpoint.front().x);
  double lane_xm_per_pix = 3.7 / 700;
  double veh_pos = (((rpoint.front().x + lpoint.front().x) * lane_xm_per_pix) / 2);
  double cen_pos = ((src.cols * lane_xm_per_pix) / 2);
  vehicle_offset = veh_pos - cen_pos;
  // imshow("show_point", show_point);
  // waitKey(1);
}

void LaneLineDetection::drawArea(const Mat &src, Mat &dst,
                                 vector<Point> &lp, vector<Point> &rp,
                                 const Mat &Minv, double &vehicle_offset,
                                 double &curvature)
{
  vector<Point> rflip, ptr;
  Mat colormask = Mat::zeros(src.rows, src.cols, CV_8UC3);
  Mat midst;
  //绘制车道线
  polylines(colormask, lp, false, Scalar(0, 255, 0), 5);
  polylines(colormask, rp, false, Scalar(0, 0, 255), 5);

  //反转坐标，以便绘制填充区域
  flip(rp, rflip, 1);
  //拼接坐标
  hconcat(lp, rflip, ptr);
  //绘制填充区域
  const Point *em[1] = {&ptr[0]};
  int nop = (int)ptr.size();
  fillPoly(colormask, em, &nop, 1, Scalar(0, 255, 0));
  //反变形
  warpPerspective(colormask, midst, Minv, src.size(), INTER_LINEAR);
  //将车道线图片和原始图片叠加
  addWeighted(src, 1, midst, 0.3, 0, dst);

  // cv::Rect roi_rect = cv::Rect(100, 0, left_curve_img.cols, left_curve_img.rows);
  // left_curve_img.copyTo(dst(roi_rect));
  // rectangle(dst, Rect(0, 0, 400, 500), Scalar(0, 0, 255), 1, LINE_8, 0);

  std::stringstream ss1;
  ss1 << std::setiosflags(std::ios::fixed) << std::setprecision(2) << vehicle_offset;
  std::string str1 = ss1.str();

  putText(dst, "Vehicle is " + str1 + "m away from center",
          Point(10, 150), FONT_HERSHEY_SIMPLEX,
          0.66, Scalar(255, 255, 255), 2);

  std::stringstream ss2;
  ss2 << std::setiosflags(std::ios::fixed) << std::setprecision(0) << curvature;
  std::string str2 = ss2.str();

  putText(dst, "Curvature = " + str2 + " m",
          Point(10, 100), FONT_HERSHEY_SIMPLEX,
          0.66, Scalar(255, 255, 255), 2);

  // imshow("colormask", colormask);
  // waitKey(1);
}
