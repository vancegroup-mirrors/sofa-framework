#ifndef SOFA_GUI_BATCHGUI_H
#define SOFA_GUI_BATCHGUI_H

#include <sofa/gui/SofaGUI.h>

namespace sofa
{

namespace gui
{

class BatchGUI : public SofaGUI
{

public:

    /// @name methods each GUI must implement
    /// @{

    BatchGUI();

    void setScene(sofa::simulation::tree::GNode* groot, const char* filename="");

    int mainLoop();
    void redraw();
    int closeGUI();

    sofa::simulation::tree::GNode* currentSimulation();

    /// @}

    /// @name registration of each GUI
    /// @{

    static int InitGUI(const char* name, const std::vector<std::string>& options);
    static SofaGUI* CreateGUI(const char* name, const std::vector<std::string>& options, sofa::simulation::tree::GNode* groot = NULL, const char* filename = NULL);

    /// @}

protected:
    /// The destructor should not be called directly. Use the closeGUI() method instead.
    ~BatchGUI();

    sofa::simulation::tree::GNode* groot;
    std::string filename;
    int nbIter;
};

} // namespace gui

} // namespace sofa

#endif
