/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc., 51  *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                   *
*******************************************************************************
*                            SOFA :: Applications                             *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#ifndef SOFA_MODELER_H
#define SOFA_MODELER_H


#include "Modeler.h"
#include "GraphModeler.h"
#include <map>
#include <vector>
#include <string>
#include <sofa/helper/Factory.h>

#include <sofa/gui/SofaGUI.h>
#include <sofa/gui/qt/RealGUI.h>

#ifdef SOFA_QT4
#include <Q3ListView>
#include <Q3TextDrag>
#include <QPushButton>
#include <QTabWidget>
#include <QTabBar>
#include <Q3PopupMenu>
#include <QAction>
#include <QComboBox>
#include <Q3Process>
#else
#include <qlistview.h>
#include <qdragobject.h>
#include <qpushbutton.h>
#include <qtabwidget.h>
#include <qtabbar.h>
#include <qpopupmenu.h>
#include <qaction.h>
#include <qcombobox.h>
#include <qprocess.h>
typedef QProcess Q3Process;
#endif


namespace sofa
{

namespace gui
{

namespace qt
{

#ifndef SOFA_QT4
typedef QListView Q3ListView;
typedef QPopupMenu Q3PopupMenu;
#endif

typedef sofa::core::ObjectFactory::ClassEntry ClassInfo;
typedef sofa::core::ObjectFactory::Creator    ClassCreator;

using sofa::simulation::tree::GNode;


class SofaModeler : public ::Modeler
{

    Q_OBJECT
public :

    SofaModeler();
    ~SofaModeler() {};

    /// Create a new empty Tab
    void createTab();
    /// Change the content of the description box. Happens when the user has clicked on a component
    void changeComponent(ClassInfo *currentComponent);
    void fileOpen(std::string filename);
    void fileSave(std::string filename);

    /// Change the name of the main window
    void changeNameWindow(std::string filename);
    /// From the name of the type of a component, gives serveral information
    ClassInfo* getInfoFromName(std::string name);
    /// Update the menu Recently Opened Files...
    void updateRecentlyOpened(std::string fileLoaded);

signals:
    void loadPresetGraph(std::string);

public slots:
    /// When dropping a dragged element, this method set the button pushed to its initial state
    void releaseButton();
    /// Change the state of the Undo button
    void updateUndo(bool v) {this->editUndoAction->setEnabled(v);}
    /// Change the state of the Redo button
    void updateRedo(bool v) {this->editRedoAction->setEnabled(v);}
    /// Change the content of the description box. Happens when the user has clicked on a component
#ifdef SOFA_QT4
    void changeInformation(Q3ListViewItem *);
#else
    void changeInformation(QListViewItem *);
#endif
    /// Change the main library label, it happens when the user open a new class of component from the library
    void changeLibraryLabel(int index);
    /// Dropping a Node in the Graph
    void newGNode();
    /// Dropping a component in the Graph
    void newComponent();

    //File Menu
    /// Creation of a new scene (new tab will be created)
    void fileNew() {fileNew(NULL);};
    void fileNew(GNode* root);

    /// Open an existing simulation (new tab will be created)
    void fileOpen();
    void fileOpen(const QString &filename) {fileOpen(std::string(filename.ascii()));}

    /// Save the current simulation
    void fileSave();
    void fileSaveAs();

    /// Remove all components of the current simulation
    void clearTab();
    /// Close the current simulation
    void closeTab();
    /// Create a new tab containing an empty simulation (by default the collision pipeline is added)
    void newTab();

    /// Quit the Modeler
    void fileExit() {close();};

    /// Launch the current simulation into Sofa
    void runInSofa();
    void sofaExited();
    void removeTemporaryFiles(Q3Process *p);

    /// Change of simulation by changing the current opened tabulation
    void changeCurrentScene( QWidget*);
    /// Change of simulation by changing the current opened tabulation
    void changeCurrentScene( int n);

    /// Propagate the action Undo to the graph
    void editUndo() {graph->editUndo();}
    /// Propagate the action Redo to the graph
    void editRedo() {graph->editRedo();}


    void editCut();
    void editCopy();
    void editPaste();

    /// Load a preset stored in the menu preset: add a node to the current simulation
    void loadPreset(int);

    /// When the user enter the Modeler, grabbing something: determine the acceptance or not
    void dragEnterEvent( QDragEnterEvent* event);
    /// When the user move the mouse around, with something grabbed
    void dragMoveEvent( QDragMoveEvent* event);

    /// Action to perform when the user drop something in the Modeler
    void dropEvent(QDropEvent* event);

    /// Open a recently Opened files from the menu Recently Opened Files...
    void fileRecentlyOpened(int id);

    /// Filter in the library all the components containing the text written
    void searchText(const QString&);

    void changeSofaBinary();
    void GUIChanged();
    //When the window is closed: we close all the Sofa launched, and remove temporary files
    void closeEvent ( QCloseEvent * e );

    ///display the plugin manager window, to add/remove some external dynamic libraries
    void showPluginManager();

    ///update the list of components available in Sofa
    void updateComponentList();

protected:
    /// Widget containing all the graphs
    QTabWidget *sceneTab;
    /// Current in-use graph
    GraphModeler *graph; //currentGraph in Use
    /// Current opened Tab
    QWidget *tabGraph;
    /// Menu runSofa for the GUI
    std::vector< QAction* > listActionGUI;
    /// Menu preset
    Q3PopupMenu *preset;
    /// Menu containing the opened simulations in the Modeler
    Q3PopupMenu *windowMenu;
    /// Correspondance between a name clicked in the menu and a path to the preset
    std::map< std::string, std::string > mapPreset;
    /// Is ready to do a paste operation?
    bool isPasteReady;

    /// plugin manager window
    SofaPluginManager* pluginManager;


    /// Main Sofa Ressources: contains all the component, with many info, and creators
    typedef std::map<  const QObject* , std::pair<ClassInfo*, QObject*> >::const_iterator libraryIterator;
    std::map<  const QObject* , std::pair<ClassInfo*, QObject*> > mapComponents;
    /// Number of components currently displayed in the library
    unsigned int displayComponents;
    /// Map between a tabulation from the modeler to an object of type GraphModeler
    std::map<  const QWidget*, GraphModeler*> mapGraph;
    /// Map between a tabulation from the modeler to a Sofa Application
    std::multimap<  const QWidget*, Q3Process*> mapSofa;
    /// Map between an index of tabulation to the tabulation itself
    std::map< int, QWidget*> mapWindow;
    /// Map between a widget corresponding to a tab of the library, and a pair of button and combo box, corresponding to a component and its template
    std::vector< std::multimap< QWidget*, std::pair< QPushButton*, QComboBox*> > > pages;
private:
    std::string sofaBinary;
    std::string presetPath;
    std::string examplePath;
    std::string binPath;
    char count;
    std::vector< QString > exampleFiles;
};
}
}
}
#endif
