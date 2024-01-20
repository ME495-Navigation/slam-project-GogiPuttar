#include <iostream>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <cmath>

using turtlelib::Transform2D;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;

int main()
{
    Transform2D Tab, Tbc;

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> Tbc;
    Transform2D Tba{Tab.inv().translation(), Tab.inv().rotation()};
    Transform2D Tcb{Tbc.inv().translation(), Tbc.inv().rotation()};
    Transform2D Tac{(Tab*Tbc).translation(), (Tab*Tbc).rotation()};
    Transform2D Tca{Tac.inv().translation(), Tac.inv().rotation()};
    std::cout << "T_{a, b}: " << Tab << "\n";
    std::cout << "T_{b, a}: " << Tba << "\n";
    std::cout << "T_{b, c}: " << Tbc << "\n";
    std::cout << "T_{c, b}: " << Tcb << "\n";
    std::cout << "T_{a, c}: " << Tac << "\n";
    std::cout << "T_{c, a}: " << Tca << std::endl;
    // TODO: Draw each frame in svg.

    Point2D pa{}, pb{}, pc{};
    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> pa;

    pb = Tba(pa);
    pc = Tca(pa);

    std::cout << "p_a " << pa << "\n";
    std::cout << "p_b " << pb << "\n";
    std::cout << "p_c " << pc << std::endl;
    // TODO: Draw each point in svg.

    Vector2D vb{}, vb_hat{}, va{}, vc{};

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> vb;

    double vb_norm = sqrt(vb.x * vb.x + vb.y * vb.y);

    if(vb_norm == 0.0)
    {
        std::cout << "INVALID VECTOR." << std::endl;
    }
    else
    {
        vb_hat.x = vb.x / vb_norm;
        vb_hat.y = vb.y / vb_norm;
    }

    va = Tab(vb);
    vc = Tcb(vb);
    
    std::cout << "v^_b " << vb_hat << "\n";
    std::cout << "v_a " << va << "\n";
    std::cout << "v_b " << vb << "\n";
    std::cout << "v_c " << vc << std::endl;
    // TODO: Draw each vector in svg.

    Twist2D Va, Vb, Vc;

    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> Vb;
    Va = Tab(Vb);
    Vc = Tcb(Vb);
    std::cout << "V_a " << Va << "\n";
    std::cout << "V_b " << Vb << "\n";
    std::cout << "V_c " << Vc << std::endl;

    return 0;
}