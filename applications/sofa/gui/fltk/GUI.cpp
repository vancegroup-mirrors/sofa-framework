/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This program is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU General Public License as published by the Free   *
* Software Foundation; either version 2 of the License, or (at your option)    *
* any later version.                                                           *
*                                                                              *
* This program is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for     *
* more details.                                                                *
*                                                                              *
* You should have received a copy of the GNU General Public License along with *
* this program; if not, write to the Free Software Foundation, Inc., 51        *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                    *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
// generated by Fast Light User Interface Designer (fluid) version 1.0105

#include "GUI.h"

namespace sofa
{

namespace gui
{

namespace fltk
{

inline void UserInterface::cb_START_i(Fl_Button* o, void*)
{
	if (viewer->GetScene()->getAnimate() == false)
	{
		o->color((Fl_Color) 190);
		o->label("STOP");
		//ComputeDeformationCB();
		viewer->GetScene()->setAnimate(true);
	}
	else
	{
		viewer->GetScene()->setAnimate(false);
		o->color((Fl_Color) 19);
		o->label("START");
	};
}

void UserInterface::cb_START(Fl_Button* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->parent()->user_data()))->
	cb_START_i(o, v);
}

inline void UserInterface::cb_Reset_i(Fl_Button*, void*)
{
	//viewer->GetScene()->updatePosition();
	//viewer->Animate();
	viewer->ResetScene();
}

void UserInterface::cb_Reset(Fl_Button* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->parent()->user_data()))->
	cb_Reset_i(o, v);
}

inline void UserInterface::cb_Preset_i(Fl_Button*, void*)
{
	viewer->SwitchToPresetView();
}

void UserInterface::cb_Preset(Fl_Button* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->parent()->user_data()))->
	cb_Preset_i(o, v);
}

inline void UserInterface::cb_Smooth_i(Fl_Check_Button* o, void*)
{
	SmoothOnOffCB(o->value()!=0);
}

void UserInterface::cb_Smooth(Fl_Check_Button* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->parent()->user_data()))->
	cb_Smooth_i(o, v);
}

inline void UserInterface::cb_Visual_i(Fl_Check_Button* o, void*)
{
	VisualOnOffCB(o->value()!=0);
}

void UserInterface::cb_Visual(Fl_Check_Button* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->parent()->user_data()))->
	cb_Visual_i(o, v);
}

inline void UserInterface::cb_Behavior_i(Fl_Check_Button* o, void*)
{
	BehaviorOnOffCB(o->value()!=0);
}

void UserInterface::cb_Behavior(Fl_Check_Button* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->parent()->user_data()))->
	cb_Behavior_i(o, v);
}

inline void UserInterface::cb_Collision_i(Fl_Check_Button* o, void*)
{
	CollisionOnOffCB((o->value()!=0));
}

void UserInterface::cb_Collision(Fl_Check_Button* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->parent()->user_data()))->
	cb_Collision_i(o, v);
}

inline void UserInterface::cb_Mapping_i(Fl_Check_Button* o, void*)
{
	MappingOnOffCB(o->value()!=0);
}

void UserInterface::cb_Mapping(Fl_Check_Button* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->parent()->user_data()))->
	cb_Mapping_i(o, v);
}

inline void UserInterface::cb_Force_i(Fl_Value_Slider* o, void*)
{
	ChangeForceIntensityCB((double) o->value());
}

void UserInterface::cb_Force(Fl_Value_Slider* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->user_data()))->cb_Force_i(o, v);
}

// Adding Thread During Simulation

inline void	UserInterface::cb_AddThread_i(Fl_Button*, void*)
{
	AddThreadCB();
}

void UserInterface::cb_AddThread(Fl_Button* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->parent()->user_data()))->
	cb_AddThread_i(o, v);
}

//

void UserInterface::show()
{
	mainWindow->show();
}

UserInterface::UserInterface(GNode* groot)
{
	Fl_Double_Window*	w; {
		Fl_Double_Window*	o	= mainWindow = new Fl_Double_Window(1090, 811,
																	"Open Framework for Medical Simulation - (C) CIMIT Simulation Group & LIFL Alc\
ove Group & INRIA Rhone-Alpes Evasion Group - 2004, 2005, 2006");
		w = o;
		o->user_data((void *) (this));
		o->align(FL_ALIGN_CENTER | FL_ALIGN_INSIDE); {
			Fl_Group*	o	= viewGroup = new Fl_Group(5, 0, 1075, 805); {
				Fl_Box*	o	= frame = new Fl_Box(145, 1, 915, 762);
				Fl_Group::current()->resizable(o);
			} {
				FLTKviewer*	o	= viewer = new FLTKviewer(140, 10, 935, 740,
														  "GL Window");
				o->box(FL_NO_BOX);
				o->color(FL_BACKGROUND_COLOR);
				o->selection_color(FL_BACKGROUND_COLOR);
				o->labeltype(FL_NORMAL_LABEL);
				o->labelfont(0);
				o->labelsize(14);
				o->labelcolor(FL_BLACK);
				o->align(FL_ALIGN_CENTER | FL_ALIGN_INSIDE);
				o->when(FL_WHEN_RELEASE);
				viewer->SetScene(groot);
			} {
				Fl_Group*	o	= optionsGroup = new Fl_Group(5, 5, 130, 780); {
					Fl_Menu_Button*	o	= new Fl_Menu_Button(5, 5, 125, 35,
															 "Files");
					o->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
				} {
					Fl_Button*	o	= new Fl_Button(7, 181, 123, 24, "START");
					o->callback((Fl_Callback *) cb_START);
				} {
					Fl_Output* o = new Fl_Output(5, 259, 25, 25, "DT:");
					o->box(FL_NO_BOX);
				} {
					dt = new Fl_Float_Input(35, 259, 95, 25, "DT:");
					char buf[16];
					//added by Sylvere F.
					sprintf(buf, "%f", viewer->GetScene()->getDt());
					//snprintf(buf,sizeof(buf),"%f",viewer->GetScene()->getDt());
					while (buf[0] && buf[strlen(buf)-1]=='0') buf[strlen(buf)-1]='\0';
					dt->value(buf);
					dt->callback((Fl_Callback *) cb_Dt);
				} {
					Fl_Button*	o	= new Fl_Button(5, 319, 125, 25,
													"Reset Scene");
					o->callback((Fl_Callback *) cb_Reset);
				} {
					Fl_Button*	o	= new Fl_Button(5, 349, 125, 25,
													"Preset View");
					o->callback((Fl_Callback *) cb_Preset);
				} {
					Fl_Check_Button*o	= new Fl_Check_Button(5, 390, 125, 25,
															  "Visual Model");
					o->down_box(FL_DOWN_BOX);
					o->value(viewer->GetScene()->getShowVisualModels());
					o->callback((Fl_Callback *) cb_Visual);
				} {
					Fl_Check_Button*o	= new Fl_Check_Button(5, 420, 125, 25,
															  "Behavior Model");
					o->down_box(FL_DOWN_BOX);
					o->value(viewer->GetScene()->getShowBehaviorModels());
					o->callback((Fl_Callback *) cb_Behavior);
				} {
					Fl_Check_Button*o	= new Fl_Check_Button(5, 450, 125, 25,
															  "Collision Model");
					o->down_box(FL_DOWN_BOX);
					o->value(viewer->GetScene()->getShowCollisionModels());
					o->callback((Fl_Callback *) cb_Collision);
				} {
					Fl_Check_Button*o	= new Fl_Check_Button(5, 480, 125, 25,
															  "Mapping");
					o->down_box(FL_DOWN_BOX);
					o->value(viewer->GetScene()->getShowMappings());
					o->callback((Fl_Callback *) cb_Mapping);
				} {
					Fl_Check_Button*o	= new Fl_Check_Button(5, 510, 125, 25,
															  "Force Field");
					o->down_box(FL_DOWN_BOX);
					o->value(viewer->GetScene()->getShowForceFields());
					o->callback((Fl_Callback *) cb_Smooth);
				} {
					Fl_Button*	o	= new Fl_Button(5, 550, 125, 25,
													"Add Thread");
					o->callback((Fl_Callback *) cb_AddThread);
				} {
					threadLabel = new Fl_Output(5, 580, 125, 25, "");
				}{
					fpsLabel = new Fl_Output(5, 610, 125, 25, "");
				}
				o->end();
			} {
				Fl_Value_Slider*o	= new Fl_Value_Slider(285, 775, 635, 20,
														  "Force magnitude (%)");
				o->type(5);
				o->minimum(-100);
				o->maximum(100);
				o->step(0.1);
				o->callback((Fl_Callback *) cb_Force);
				o->align(FL_ALIGN_LEFT);
				o->value(0.0);
			}
			o->end();
			Fl_Group::current()->resizable(o);
		}
		o->end();
	}
}

inline void UserInterface::cb_Dt_i(Fl_Float_Input* o, void*)
{
	DtCB(atof(o->value()));
}

void UserInterface::cb_Dt(Fl_Float_Input* o, void* v)
{
	((UserInterface *) (o->parent()->parent()->parent()->user_data()))->
	cb_Dt_i(o, v);
}

} // namespace fltk

} // namespace gui

} // namespace sofa
