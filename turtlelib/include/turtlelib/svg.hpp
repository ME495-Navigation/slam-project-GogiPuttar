// SVG.h
#ifndef SVG_H
#define SVG_H

#include <iostream>
#include <fstream>
#include <vector>

class SVG {
public:
    SVG(int width, int height);
    void addRectangle(int x, int y, int width, int height);
    void addCircle(int cx, int cy, int r, const std::string& color);
    void addLine(int x1, int y1, int x2, int y2, const std::string& stroke);
    void addText(int x, int y, const std::string& text);
    void saveToFile(const std::string& filename, const std::string& additionalContent);
    int getWidth();

private:
    int width;
    int height;
    std::vector<std::string> elements;
};

#endif // SVG_H
