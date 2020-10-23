Email

<eowjd4@naver.com>

My resume

<http://eowjd0512.github.io/>

# VITAMIN-E
Re-implementation of VITAMIN-E SLAM

This project was started to re-produce the paper

"M. Yokozuka et al, VITAMIN-E: VIsual Tracking And MappINg with Extremely Dense Feature Points, CVPR 2019"

## Requirements

1. Eigen - the latest version (>= 3.3.9) from <https://gitlab.com/libeigen/eigen>

2. OpenCV (we tested at the version 4.4.0)

3. Panglin - <https://github.com/stevenlovegrove/Pangolin>

## Note

### Progress

Implementing the architecture...

### Architecture

We used the architecture on the ORB-SLAM System <https://github.com/raulmur/ORB_SLAM2>.
This is because it has a complete and efficient framework suitable for multi-thread-based SLAM. 

### Feature tracking

Thanks to yycho0108 <https://github.com/yycho0108/vitamin-e>,
we can be able to implement the feature tracking part easily. 

### code optimization

The code will be further optimized using modern C++ after implementation is complete.
