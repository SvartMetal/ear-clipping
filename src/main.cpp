#include "stdafx.h"

#include "viewer.h"

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);
    triangulation_viewer viewer;
    visualization::run_viewer(&viewer, "Ear clipping");
}
