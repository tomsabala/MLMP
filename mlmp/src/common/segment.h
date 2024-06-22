#ifndef SEGMENT_H
#define SEGMENT_H


namespace mlmp {
    namespace common {
        struct Segment
        {
            Segment(double p0_x, double p0_y, double p1_x, double p1_y) : x0(p0_x), y0(p0_y), x1(p1_x), y1(p1_y)
            {
            }
            double x0, y0, x1, y1;

            void print() {
                std::cout <<x0 << " " << y0 << " -> " << x1 << " "<< y1<<std::endl;
            }
        };

        struct Point
        {
            Point(double p_x, double p_y) : x(p_x), y(p_y)
            {
            }
            double x, y;

            void print() {
                std::cout <<x << " " << y <<std::endl;
            }
        };
    }
}

#endif