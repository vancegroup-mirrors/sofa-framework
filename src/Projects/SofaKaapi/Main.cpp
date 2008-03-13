#include <iostream>
#include <fstream>
#include <stack>

#include <kaapi>
#include <kaapi_closure.h>
#include <sofa/simulation/tree/ParallelActionScheduler.h>
#include <sofa/simulation/tree/GNode.h>
#include <sofa/simulation/tree/CactusStackStorage.h>
#include <sofa/simulation/tree/Action.h>
#include <sofa/simulation/tree/Simulation.h>
#include <sofa/gui/qt/Main.h>

#include <sofa/helper/system/thread/CTime.h>
#include <sofa/helper/Factory.h>
using
  sofa::helper::system::thread::CTime;
using
  sofa::helper::Factory;
using
  sofa::helper::system::thread::ctime_t;


using namespace sofa::simulation::tree;
//#define VERBOSE
//#define KAAPI_2_0
//KAAPI_DECL_EVENT2STRING( BU_START, 1, 6, long, threadid,  truc,40 );
//KAAPI_DECL_EVENT2STRING( BU_END, 1, 7, long, threadid, truc,40 );
//KAAPI_DECL_EVENT2STRING( TD_START, 1, 8, long, threadid,  truc,40 );
//KAAPI_DECL_EVENT2STRING( TD_END, 1, 9, long, threadid,  truc,40 );
#define TASKBOTTOMUP
//#define KAAPI_LOG_EVENT2(a,b,c,d) std::cout<<c<<std::endl
// ---------------------------------------------------------------------
// --- SOFA+KAAPI integration
// ---------------------------------------------------------------------
namespace
  SofaKaapi
{


// First the SOFA encapsulation classes...

  class
    KaapiScheduler:
    public
    ParallelActionScheduler
  {

  public:
  KaapiScheduler (bool propagate = false):ParallelActionScheduler
      (propagate)
    {

    }


  protected:
    virtual
      ParallelActionScheduler *
    clone ()
    {

      return new KaapiScheduler (*this);

    }


    void
    executeParallelAction (GNode * node, Action * action);

  };


// Then the Kaapi encapsulation classes...

/** SofaTopDownActionTask
 */
  class
    SofaTopDownActionTask:
    public
    RFO::Closure
  {

  public:
    GNode *
      node;

    Action *
      action;


    CactusStackStorage *
      data;


#ifdef TASKBOTTOMUP
    DFG::Access
      res;

#endif				/* 
				 */
    static void
    doit (RFO::Closure * c, Core::Thread * t);

  };



  class
    ClosureFormatSofaTopDownActionTask:
    public
    DFG::WrapperClosureFormat <
    SofaTopDownActionTask >
  {
  public:
    ClosureFormatSofaTopDownActionTask ():
      DFG::WrapperClosureFormat <
    SofaTopDownActionTask >
      ("SofaTopDownActionTask", &SofaTopDownActionTask::doit)
    {
    }


    /** Return true iff the closure is ready for execution
	 */
    bool
    is_ready (RFO::Closure * /*c */ ) const
    {
      return
	true;
    }


    /** Return the size in bytes
	 */
    size_t
    get_size (const RFO::Closure * /*c */ ) const
    {
      return
      sizeof (SofaTopDownActionTask);
    }


    /** Get the number of parameters
	 */
    int
    get_nparam (const RFO::Closure * /*c */ ) const
    {

#ifdef TASKBOTTOMUP
      return
	1;

#else				/* 
				 */
      return
	0;

#endif				/* 
				 */
    }


    /** Get the format of the parameter i
	 */
#ifdef KAAPI_2_0
    const
      Net::Format *
    get_fmtparam (const RFO::Closure * /*c */ , int /*i */ ) const
    {

      return &
	Net::FormatDef::Access;

    }

#else				/* 
				 */
    const
      Util::Format *
    get_fmtparam (const RFO::Closure * /*c */ , int /*i */ ) const
    {

      return &
	Util::FormatDef::Access;

    }

#endif				/* 
				 */

    /** Get the pointer to the parameter i
	 */
#ifdef TASKBOTTOMUP
    void *
    get_param (const RFO::Closure * c, int /*i */ ) const
    {

      SofaTopDownActionTask *
	clo = (SofaTopDownActionTask *) c;

      return &
	clo->
	res;

    }

#else				/* 
				 */
    void *
    get_param (const RFO::Closure * /*c */ , int /*i */ ) const
    {

      return
	NULL;

    }

#endif				/* 
				 */

    /** Get the mode of access to the parameter i
	*/
    DFG::AccessMode::Val
    get_mode (const RFO::Closure * /*c */ , int /*i */ ) const
    {

      return
	DFG::AccessMode::w;

    }


    /** Return true if the i-th parameter is an Access
     */
    bool
    is_access (const RFO::Closure * /*c */ , int /*i */ ) const
    {
      return
	true;
    }

  };

  static
    ClosureFormatSofaTopDownActionTask
    formatSofaTopDownActionTask;


#ifdef TASKBOTTOMUP
/** SofaBottomUpActionTask
 */
  class
    SofaBottomUpActionTask:
    public
    RFO::Closure
  {

  public:
    DFG::Access
      res;

    std::vector <
      DFG::Access >
      inputs;

    Action *
      action;

    GNode **
      nodes;



    CactusStackStorage *
      datas;

  SofaBottomUpActionTask (int n = 1):inputs (n), action (NULL), nodes (NULL),
      datas (NULL)
    {

    }
    static void
    doit (RFO::Closure * c, Core::Thread * t);

  };

  class
    ClosureFormatSofaBottomUpActionTask:
    public
    DFG::WrapperClosureFormat <
    SofaBottomUpActionTask >
  {

  public:
    ClosureFormatSofaBottomUpActionTask ():
      DFG::WrapperClosureFormat <
    SofaBottomUpActionTask >
      ("SofaBottomUpActionTask", &SofaBottomUpActionTask::doit)
    {
    }


    /** Return true iff the closure is ready for execution
	 */
    //bool is_ready(RFO::Closure* c) const
    //{ return true; }

    /** Return the size in bytes
	 */
    size_t
    get_size (const RFO::Closure * /*c */ ) const
    {
      return
      sizeof (SofaBottomUpActionTask);
    }


    /** Get the number of parameters
	 */
    int
    get_nparam (const RFO::Closure * c) const
    {

      SofaBottomUpActionTask *
	clo = (SofaBottomUpActionTask *) c;

      return
	1 +
	clo->
	inputs.
      size ();

    }


    /** Get the format of the parameter i
	 */
#ifdef KAAPI_2_0
    const
      Net::Format *
    get_fmtparam (const RFO::Closure * /*c */ , int /*i */ ) const
    {

      return &
	Net::FormatDef::Access;

    }

#else				/* 
				 */
    const
      Util::Format *
    get_fmtparam (const RFO::Closure * /*c */ , int /*i */ ) const
    {

      return &
	Util::FormatDef::Access;

    }

#endif				/* 
				 */

    /** Get the pointer to the parameter i
	 */
    void *
    get_param (const RFO::Closure * c, int i) const
    {

      SofaBottomUpActionTask *
	clo = (SofaBottomUpActionTask *) c;

      if (i == 0)

	return &clo->res;

      else

	return &(clo->inputs[i - 1]);

    }


    /** Get the mode of access to the parameter i
	 */
    DFG::AccessMode::Val
    get_mode (const RFO::Closure * /*c */ , int i) const
    {

      if (i == 0)

	return DFG::AccessMode::w;

      else

	return
	  DFG::AccessMode::r;

    }


    /** Return true if the i-th parameter is an Access
	 */
    bool
    is_access (const RFO::Closure * /*c */ , int /*i */ ) const
    {
      return
	true;
    }

  };

  static
    ClosureFormatSofaBottomUpActionTask
    formatSofaBottomUpActionTask;

#endif /* 
        */

// And now the CODE!!!

  bool
    dump2dot = false;

Util::Atomic dotcount;


  void
  SofaTopDownActionTask::doit (RFO::Closure * c, Core::Thread * t)
  {
//  std::cout<<"  SofaTopDownActionTask::doit: "<<pthread_self()<<std::endl;

    DFG::Thread * thread = static_cast < DFG::Thread * >(t);

    SofaTopDownActionTask *
      clo = static_cast < SofaTopDownActionTask * >(c);

    GNode *
      node = clo->node;

    Action *
      action = clo->action;
  std::string actionName = gettypename (typeid (*action));

//  KAAPI_LOG_EVENT2( Util::Thread::get_current(),TD_START,(long)pthread_self(),(char *)actionName.c_str());


    CactusStackStorage *
      data = clo->data;


#ifdef TASKBOTTOMUP
    // res is just the recursion level
    long *
      ires = (long *) clo->res.get_gd ()->get_data ();

    ++*ires;

#endif /* 
        */

    Action::Result res = action->processNodeTopDown (node, data);

    if (res == Action::RESULT_CONTINUE && !node->child.empty ())

      {

	RFO::Frame frame;

	thread->push (&frame);


	const int
	  n = node->child.size ();

	const int
	  dataSize = sizeof (double);
	//action->getLocalStorageSize ();


	SofaTopDownActionTask
	  child[n];

	GNode **
	  childnode = new GNode *[n];
	CactusStackStorage *
	  childdata = new CactusStackStorage[n];

	char
	  databuffer[n * dataSize];

#ifdef TASKBOTTOMUP
	long
	  childres[n];

	DFG::GlobalData kres[n];

	DFG::GlobalData::Attribut attr;
	
	attr.set_sticky ();

#endif /* 
        */
	for (int i = 0; i < n; i++)

	  {

#ifdef TASKBOTTOMUP
	    childres[i] = *ires;

#ifdef KAAPI_2_0
	    kres[i].initialize (&(childres[i]), &Net::FormatDef::Long, attr);

#else /* 
       */
	    kres[i].initialize (&(childres[i]), &Util::FormatDef::Long, attr);

#endif /* 
        */
	    child[i].res.link (&(child[i]), &(kres[i]), DFG::AccessMode::w);

#endif /* 
        */
	    child[i].action = action;

	    child[i].node = childnode[i] = node->child[i];

	    child[i].data = &(childdata[i]);

//child[i].parentData = data;   //
	    childdata[i].setParent (data);
	    frame.push (&(child[i]), &formatSofaTopDownActionTask,
			&SofaTopDownActionTask::doit);

	  }

#ifdef TASKBOTTOMUP
	SofaBottomUpActionTask
	sum (n);

	sum.action = action;



	sum.datas = childdata;

	sum.nodes = childnode;

	sum.res.link (&sum, &clo->res, DFG::AccessMode::w);

	for (int i = 0; i < n; i++)

	  {

	    sum.inputs[i].link (&sum, &child[i].res, DFG::AccessMode::r);

	  }
	frame.push (&sum, &formatSofaBottomUpActionTask,
		    &SofaBottomUpActionTask::doit);

#endif /* 
        */
	if (dump2dot)

	  {

	    int
	      count = dotcount.incr_and_return ();

	    std::string actionName = gettypename (typeid (*action));

	    std::string nodeName = node->getName ();

	    std::ostringstream ofilename;

	    ofilename << "dfg" << "-" << count << "-" << actionName << "-" <<
	      nodeName << ".dot";

	    std::string filename = ofilename.str ();

	    std::cout << filename << std::endl;

	    std::ofstream fout (filename.c_str ());

	    DFG::ODotStream odot (&fout);

	    thread->dump (odot);

	  }


	for (int i = 0; i < n; i++)

	  {

	    thread->execute (&frame, &child[i]);

	  }
#ifdef TASKBOTTOMUP
	thread->execute (&frame, &sum);

	thread->pop ();

#else /* 
       */
	thread->pop ();

	for (int i = 0; i < n; i++)

	  {

	    action->processNodeBottomUp (childnode[i], childdata[i], data);

	  }
#endif /* 
        */
      }
//  KAAPI_LOG_EVENT2( Util::Thread::get_current(),TD_END,(long)pthread_self(),(char *)actionName.c_str());

  }

#ifdef TASKBOTTOMUP
  void
  SofaBottomUpActionTask::doit (RFO::Closure * c, Core::Thread * /*t */ )
  {

//  std::cout<<" BottomUpAction: "<<pthread_self()<<std::endl;
    //DFG::Thread* thread = static_cast<DFG::Thread*>(t);
    SofaBottomUpActionTask *
      clo = static_cast < SofaBottomUpActionTask * >(c);


    Action *
      action = clo->action;
  std::string actionName = gettypename (typeid (*action));
//  KAAPI_LOG_EVENT2( Util::Thread::get_current(),BU_START,(long)pthread_self(),(char *)actionName.c_str());

    const int
      n = clo->inputs.size ();


    GNode **
      nodes = clo->nodes;

    CactusStackStorage *
      datas = clo->datas;

    for (int i = 0; i < n; i++)

      {

	action->processNodeBottomUp (nodes[i], &(datas[i]));

      }
    delete[]nodes;
    delete[]datas;
//  KAAPI_LOG_EVENT2( Util::Thread::get_current(),BU_END,(long)pthread_self(),(char *)actionName.c_str());
  }
#endif /* 
        */

  void
  KaapiScheduler::executeParallelAction (GNode * node, Action * action)
  {

#ifdef VERBOSE
    std::string actionName = gettypename (typeid (*action));

    std::string nodeName = node->getPathName ();

    std::
      cout << ">Kaapi action task " << actionName << " on " << nodeName <<
      pthread_self () << std::endl;

#endif /* 
        */
    DFG::Thread * thread =
      dynamic_cast < DFG::Thread * >(Core::Thread::get_current ());

    RFO::Frame frame;

    thread->push (&frame);


#ifdef TASKBOTTOMUP
    long
      input = 0;

    long
      output = 0;

    DFG::GlobalData sinput;

    DFG::GlobalData soutput;

    DFG::GlobalData::Attribut attr;

    attr.set_sticky ();

#ifdef KAAPI_2_0
    sinput.initialize (&input, &Net::FormatDef::Long, attr);

    soutput.initialize (&output, &Net::FormatDef::Long, attr);

#else /* 
       */
    sinput.initialize (&input, &Util::FormatDef::Long, attr);

    soutput.initialize (&output, &Util::FormatDef::Long, attr);

#endif /* 
        */
#endif /* 
        */

    CactusStackStorage *
      datas = new CactusStackStorage[1];
    GNode **
      nodes = new GNode *[1];

    char
    databuffer[sizeof (double)];

    nodes[0] = node;

//datas[0] = databuffer;


    SofaTopDownActionTask
      task1;

    task1.action = action;

    task1.node = node;

    task1.data = datas;

//task1.parentData = NULL;

#ifdef TASKBOTTOMUP
    task1.res.link (&task1, &sinput, DFG::AccessMode::w);

#endif /* 
        */
    frame.push (&task1, &formatSofaTopDownActionTask,
		&SofaTopDownActionTask::doit);


#ifdef TASKBOTTOMUP
    SofaBottomUpActionTask
    task2 (1);

    task2.action = action;

    task2.nodes = nodes;

    task2.datas = datas;

//task2.parentData = NULL;

    task2.inputs[0].link (&task2, &task1.res, DFG::AccessMode::r);

    task2.res.link (&task2, &soutput, DFG::AccessMode::w);

    frame.push (&task2, &formatSofaBottomUpActionTask,
		&SofaBottomUpActionTask::doit);

#endif /* 
        */

    if (dump2dot)

      {

	int
	      count = dotcount.incr_and_return ();

	std::string actionName = gettypename (typeid (*action));

	std::string nodeName = node->getName ();

	std::ostringstream ofilename;

	ofilename << "dfg" << "-" << count << "-" << actionName << "-" <<
	  nodeName << "-start.dot";

	std::string filename = ofilename.str ();

	std::cout << filename << std::endl;

	std::ofstream fout (filename.c_str ());

	DFG::ODotStream odot (&fout);

	thread->dump (odot);

      }


    thread->execute (&frame, &task1);

#ifdef TASKBOTTOMUP
    thread->execute (&frame, &task2);

    thread->pop ();

#else /* 
       */
    thread->pop ();

    action->processNodeBottomUp (node, datas[0], NULL);

#endif /* 
        */
#ifdef VERBOSE
    std::
      cout << "<Kaapi action task " << actionName << " on " << nodeName <<
      std::endl;

#endif /* 
        */
  }

}				// namespace SofaKaapi


// ---------------------------------------------------------------------
// --- MAIN
// ---------------------------------------------------------------------
int
main (int argc, char **argv)
{
  try
  {
    WS::Init::component.is_required_by ();
    Util::KaapiComponentManager::initialize (&argc, &argv);

    /* commit : start all services */
    WS::Community * com =
      dynamic_cast <
      WS::Community *
      >(Net::Group::
	resolve (Util::KaapiComponentManager::prop["community.name"]));
//sleep(100);
    if (com->is_leader ())
      {
	//std::string fileName = "../../../examples/demoChainFFD.scn";
	std::string fileName = "../../../examples/Benchmarks/Bar8-spring-implicit-32.scn";
	int
	  nbIter = 500;


	if (argc > 1)
	  fileName = argv[1];

	GNode *
	  groot = NULL;

	if (!fileName.empty ())
	  {
	    groot = Simulation::load (fileName.c_str ());
	  }

	if (groot == NULL)
	  {
	    groot = new GNode;
	  }

	std::string nthread =
	  Util::KaapiComponentManager::prop["community.thread.poolsize"];


	if (nthread != "0")

	  {

	    //=======================================
	    // Add our magic scheduler to the root node.
	    // It will replicate itself to the rest of the tree automatically
	    groot->addObject (new SofaKaapi::KaapiScheduler (3));

	  }


	//=======================================
	// Run the main loop
	std::cout << "Compution animation of " << fileName << std::endl;


	std::cout << "Computing first iteration." << std::endl;

	ctime_t
	  t = CTime::getRefTime ();


	SofaKaapi::dump2dot = false;	//true;
	SofaKaapi::dotcount.initialize ();

	SofaKaapi::dotcount.set (0);


	Simulation::animate (groot);


	SofaKaapi::dump2dot = false;


	std::
	  cout << "Computing " << nbIter << " iterations on " << nthread <<
	  " thread(s)." << std::endl;

	ctime_t
	  t0 = CTime::getRefTime ();

	int
	  n = 0;

	for (int i = 0; i < nbIter; i++)

	  {

//int
//            n2 = i * 80 / (nbIter - 1);

//while (n2 > n)

	    {

	      std::cout << '.' << std::flush;

	      ++n;

	    }

	    Simulation::animate (groot);

	  }

	std::cout << std::endl;

	ctime_t
	  t1 = CTime::getRefTime ();

	std::cout << nbIter << " iterations done." << std::endl;

	std::cout << "Time: " << ((t1 - t0) /
				  (CTime::getRefTicksPerSec () / 1000)) *
	  0.001 << " seconds, " << ((t1 - t0) /
				    (CTime::getRefTicksPerSec () / 1000)) /
	  (double) nbIter << " ms/it." << std::endl;

	std::string logname =
	  fileName.substr (0,
			   fileName.length () - 4) + "-log-" + nthread +
	  ".txt";

	std::ofstream flog (logname.c_str ());

	flog << "Time: " << ((t1 - t0) /
			     (CTime::getRefTicksPerSec () / 1000)) *
	  0.001 << " seconds, " << ((t1 - t0) /
				    (CTime::getRefTicksPerSec () / 1000)) /
	  (double) nbIter << " ms/it." << std::endl;

	flog.close ();

	//std::string objname = fileName.substr(0,fileName.length()-4)+"-scene-"+nthread+".obj";
	//std::cout << "Exporting to OBJ " << objname << std::endl;
	//Sofa::Components::Graph::Simulation::exportOBJ(groot, objname.c_str());
	std::exit (0);		// exit immediatly bypassing Kaapi termination steps
      }


    std::cout << "Main thread leave" << std::endl;

    com->leave ();

    com->terminate ();

    Util::KaapiComponentManager::terminate ();

  } catch (const Util::Exception & e)
  {

    std::cout << "[Main] catch: " << e.what () << std::endl;

  } catch (...)
  {

    std::cout << "[Main] catch an unknown exception" << std::endl;

  }
  return 0;

}
