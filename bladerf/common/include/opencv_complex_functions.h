#ifndef _OPENCV_COMPLEX_FUNCTIONS_H_
#define _OPENCV_COMPLEX_FUNCTIONS_H_

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
// need for VS for pi and other math constatnts
#define _USE_MATH_DEFINES

#elif defined(__linux__)

#endif

#include <cstdint>
#include <cmath>
#include <vector>
#include <complex>

// OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

//-----------------------------------------------------------------------------
inline void cv_decimate(cv::Mat& src, cv::Mat& dst, double rate)
{
    uint64_t num = (uint64_t)std::ceil(src.total() / rate);

    double index = 0.0;

    dst = cv::Mat::zeros(src.rows, num, src.type());

    cv::MatIterator_<double> src_itr = src.begin<double>();
    cv::MatIterator_<double> dst_itr = dst.begin<double>();
    cv::MatIterator_<double> dst_end = dst.end<double>();

    for (; dst_itr != dst_end; ++dst_itr)
    {
        *dst_itr = *(src_itr + floor(index));
        index += rate;
    }

}   // end of cv_cmplx_decimate

//-----------------------------------------------------------------------------
inline void cv_cmplx_decimate(cv::Mat& src, cv::Mat& dst, double rate)
{
    uint64_t num = (uint64_t)std::ceil(src.total() / rate);

    double index = 0.0;

    dst = cv::Mat::zeros(src.rows, num, src.type());

    cv::MatIterator_<cv::Vec2d> src_itr = src.begin<cv::Vec2d>();
    cv::MatIterator_<cv::Vec2d> dst_itr = dst.begin<cv::Vec2d>();
    cv::MatIterator_<cv::Vec2d> dst_end = dst.end<cv::Vec2d>();

    for (; dst_itr != dst_end; ++dst_itr)
    {
        *dst_itr = *(src_itr + floor(index));
        index += rate;
    }

}   // end of cv_cmplx_decimate


//-----------------------------------------------------------------------------
inline void cv_polar_discriminator(cv::Mat& src, cv::Mat &dst, double scale)
{
    std::complex<double> tmp;

    // assuming this is a row vector
    dst = cv::Mat::zeros(src.rows, src.cols-1, CV_64FC1);

    cv::MatIterator_<cv::Vec2d> src_itr = src.begin<cv::Vec2d>()+1; 
    cv::MatIterator_<cv::Vec2d> src_end = src.end<cv::Vec2d>();
    cv::MatIterator_<double> dst_itr = dst.begin<double>();

    //std::vector<T> res(v1.size() - 1);

    //auto v1_itr0 = v1.begin();
    //auto v1_itr1 = (v1.begin() + 1);
    //auto v1_end = v1.end();
    //auto res_itr = res.begin();
    //auto res_end = res.end();

    for( ; src_itr != src_end; ++src_itr, ++dst_itr)
    {
        tmp = std::complex<double>((*(src_itr-1))[0], (*(src_itr-1))[1]);
        tmp = std::complex<double>((*(src_itr))[0], (*(src_itr))[1]) * std::conj(tmp);

        *dst_itr = scale * std::atan2(tmp.imag(), tmp.real());
    }

}   // end of polar_discriminator


//-----------------------------------------------------------------------------
inline cv::Mat clamp(cv::Mat& src, double min_value, double max_value)
{
    cv::Mat dst = cv::Mat(src.size(), CV_64FC1);

    cv::MatIterator_<double> itr;
    cv::MatIterator_<double> end;
    cv::MatIterator_<double> dst_itr = dst.begin<double>();

    for (itr = src.begin<double>(), end = src.end<double>(); itr != end; ++itr, ++dst_itr)
    {
        *dst_itr = (*itr < min_value) ? min_value : ((*itr > max_value) ? max_value : *itr);
    }

    return dst;
}

//-----------------------------------------------------------------------------
inline cv::Mat cv_frequency_rotate(cv::Mat& src, double fr)
{
    uint64_t index = 0;
    std::complex<double> r;
    std::complex<double> j(0,1);

    std::complex<double> c1 = -2.0 * j * M_PI * fr;

    cv::Mat dst = cv::Mat::zeros(src.rows, src.cols, CV_64FC2);

    cv::MatIterator_<double> src_itr = src.begin<double>();
    cv::MatIterator_<double> src_end = src.end<double>();
    cv::MatIterator_<cv::Vec2d> dst_itr = dst.begin<cv::Vec2d>();

    for (; src_itr != src_end; ++src_itr, ++dst_itr)
    {
        r = std::exp(c1 * (double)index);
        (*dst_itr)[0] = *src_itr * r.real();
        (*dst_itr)[1] = *src_itr * r.imag();

        ++index;
    }

    return dst;
}

//-----------------------------------------------------------------------------
inline cv::Mat cv_cmplx_abs(cv::Mat& src)
{
    std::complex<double> tmp;
    cv::Mat dst = cv::Mat::zeros(src.rows, src.cols, CV_64FC1);

    cv::MatIterator_<cv::Vec2d> src_itr = src.begin<cv::Vec2d>();
    cv::MatIterator_<cv::Vec2d> src_end = src.end<cv::Vec2d>();
    cv::MatIterator_<double> dst_itr = dst.begin<double>();

    for (; src_itr != src_end; ++src_itr, ++dst_itr)
    {
        tmp = std::complex<double>((*src_itr)[0], (*src_itr)[1]);
        *dst_itr = std::abs(tmp);
    }

    return dst;
}
#endif  // _OPENCV_COMPLEX_FUNCTIONS_H_
