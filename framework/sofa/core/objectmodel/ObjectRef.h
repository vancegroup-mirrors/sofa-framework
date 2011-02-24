#ifndef SOFA_CORE_OBJECTMODEL_OBJECTREF_H
#define SOFA_CORE_OBJECTMODEL_OBJECTREF_H

#include <sofa/core/BaseState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/BaseContext.h>

#include <sofa/helper/vector.h>

namespace sofa
{

namespace core
{

namespace objectmodel
{

/*StateRef maybe ?? */
class ObjectRef
{
protected:
    BaseState* refObject;
    std::string refStr;

public:
    template<class RealObject>
	static RealObject* parseString( const std::string& attr,  core::objectmodel::BaseObjectDescription* arg)
    {
        RealObject* obj = NULL;

        size_t posAt = attr.find('@');

        if ( posAt != std::string::npos )
		{
			std::string tmpStr = attr.substr( posAt + 1);
			//convert sofa scene path to xml path
			tmpStr = "../" + tmpStr;
			obj = dynamic_cast<RealObject*>( arg->findObject(tmpStr.c_str() ) );
		}

        return obj;
    }

    template<class RealObject>
    static RealObject* parse( const char* XMLkeyword, core::objectmodel::BaseObjectDescription* arg )
    {
    if (arg == NULL)
    {
      return NULL;
    }
    RealObject* obj = NULL;
		const char *ptr_attr = NULL;
		ptr_attr = arg->getAttribute(XMLkeyword,NULL);

		if (ptr_attr == NULL)
			return obj;

		std::string attr(ptr_attr);
		return ObjectRef::parseString<RealObject>(attr, arg);
    }

#ifndef SOFA_DEPRECATE_OLD_API
	static std::string convertFromXMLPathToSofaScenePath(const std::string & xmlPath )
    {
		//1st Case : path ".."
		if (xmlPath == std::string("..") || xmlPath == std::string("../") )
			return ".";

		//2nd Case : path like this : "../../test"
		size_t posAt = xmlPath.find("../");
		//Note: posAt should be 0
		if(posAt == 0)
			return xmlPath.substr(3);

		//3rd : it is not a convertible xml path ...
		return xmlPath;
	}

    static Base* parseFromXMLPath( const char* XMLkeyword, core::objectmodel::BaseObjectDescription* arg )
    {
        Base* obj = NULL;
		const char *ptr_attr = NULL;
		ptr_attr = arg->getAttribute(XMLkeyword,NULL);

		if (ptr_attr == NULL)
			return obj;

		std::string attr(ptr_attr);
		obj = arg->findObject(attr.c_str()) ;

		return obj;
    }
#endif // SOFA_DEPRECATE_OLD_API

    //operator BaseState*() { return refObject; }
    template<class RealObject>
    RealObject* getObject(sofa::core::objectmodel::BaseContext* context) const
    {
        RealObject* res = NULL;

		//this method need to be const (i.e do not touch any members)
		//consequently, it is not possible to do 'update-on-read'
        //if(refObject == NULL)
        //{
        res = context->get<RealObject>(refStr);
        //    refObject = res;
        //}
        //else
        //res = dynamic_cast<RealObject*>( refObject );

        return res;
    }

	bool setPath(const std::string &path)
    {	
		size_t posAt = path.find('@');
		if(posAt == 0)
		{
			refStr = path.substr(posAt + 1);
			return true;
		}
#ifndef SOFA_DEPRECATE_OLD_API
		else
			refStr = path;

		return true;
#else
		return false;
#endif // SOFA_DEPRECATE_OLD_API
    }

	const std::string getPath() const
	{
		return refStr;
	}

    std::istream& read ( std::istream& is )
    {
		std::string tmp;
        is >> tmp;
		setPath(tmp);
        return is;
    }

    std::ostream& write ( std::ostream& os) const
    {
        os << "@" + refStr;
        return os;
    }

    inline friend std::ostream& operator<< ( std::ostream& os, const ObjectRef& ref)
    {
        return ref.write(os);
    }

    inline friend std::istream& operator>> ( std::istream& is, ObjectRef& ref)
    {
        return ref.read(is);
    }

	/// Two ObjectRefs is identical if they have the same reference (i.e the same pointer)
	inline bool operator ==( const ObjectRef& value ) const
    {
		return refObject == value.refObject;
    }

	inline bool operator !=( const ObjectRef& value ) const
    {
        return refObject != value.refObject;
    }
};


		
class SOFA_CORE_API VectorObjectRef : public std::vector<ObjectRef>
{
private:
	static const helper::vector<std::string> tokenize(const std::string &str)
	{
		helper::vector<std::string> word_s;
		std::string word;
		unsigned int it=0;
		while (str[it] ==' ') ++it;

		while(it<str.length())
		{
			if((str[it]!=' ') && (str[it]!=','))
			{
				word.push_back(str[it]);
			}
			else if(!word.empty())
			{
				word_s.push_back(word);
				word.clear();
			}

			++it;
		}
		if(!word.empty()) word_s.push_back(word);

		return word_s;
	}

public:
	///Return false if one of the object is invalid (wrong template, path, etc)
    template<class RealObject>
	static bool parseAll(const char* XMLkeyword, core::objectmodel::BaseObjectDescription* arg, helper::vector<RealObject*> &vObj)
	{
		vObj.clear();

		const char *ptr_attr = NULL;
		ptr_attr = arg->getAttribute(XMLkeyword,NULL);

		if (ptr_attr == NULL)
			return false;

		std::string attr(ptr_attr);
		const helper::vector<std::string> listStrings = VectorObjectRef::tokenize(attr);

		for(unsigned int i=0 ; i<listStrings.size() ; i++)
		{
			RealObject* obj = ObjectRef::parseString<RealObject>(listStrings[i], arg);
			if(obj == NULL){
				vObj.clear();
				return false;
			}
			vObj.push_back(obj);
		}
		return true;
	}

	std::istream& read ( std::istream& is )
    {
		std::string isStr;
		std::string subValue;
		is >> isStr;

		const helper::vector<std::string> listStrings = VectorObjectRef::tokenize(isStr);
		for(unsigned int i=0 ; i<listStrings.size() ; i++)
		{
			ObjectRef objRef;

			if (objRef.setPath(listStrings[i]))
				this->push_back(objRef);
			else
				std::cerr << "Error, " << listStrings[i] << " is not a link. (Have you forgotten the @ ?" << std::endl;
		}
        return is;
    }

    std::ostream& write ( std::ostream& os) const
    {
		for(unsigned int i = 0 ; i < this->size(); i++)
		{
			os << this->at(i) << " ";
			if(i+1 < this->size())
				os << " ";
		}

        return os;
    }

    const ObjectRef& operator[] ( unsigned int i ) const {
    	return this->at(i);
	}

    ObjectRef& operator[] ( unsigned int  i ) {
    	return this->at(i);
    }

    inline friend std::ostream& operator<< ( std::ostream& os, const VectorObjectRef& ref)
    {
        return ref.write(os);
    }

    inline friend std::istream& operator>> ( std::istream& is, VectorObjectRef& ref)
    {
        return ref.read(is);
    }

	/// Two ObjectRefs is identical if they have the same reference (i.e the same pointer)
	inline bool operator ==( const VectorObjectRef& value ) const
    {
		for(unsigned int i = 0 ; i < this->size(); i++)
		{
			if(this->at(i) != value.at(i))
				return false;
		}

		return true;
    }

	inline bool operator !=( const VectorObjectRef& value ) const
    {
		for(unsigned int i = 0 ; i < this->size(); i++)
		{
			if(this->at(i) != value.at(i))
				return true;
		}

		return false;
    }
};

class SOFA_CORE_API DataObjectRef : public sofa::core::objectmodel::Data<ObjectRef>
{
	typedef sofa::core::objectmodel::Data<ObjectRef> Inherit;

public:
	/** Constructor
        this constructor should be used through the initData() methods
     */
    explicit DataObjectRef(const BaseData::BaseInitData& init)
    : Inherit(init)
    {
    }

    /** Constructor
        this constructor should be used through the initData() methods
     */
    explicit DataObjectRef(const Inherit::InitData& init)
    : Inherit(init)
    {
    }

	void setValue(const std::string& v)
    {
		ObjectRef& objRef = *this->beginEdit();
		if(!objRef.setPath(v))
			this->getOwner()->serr << v << " is not a correct link (Have you forgotten the @ ?)" << this->getOwner()->sendl;
		this->endEdit();
    }

	std::string getValueString() const
	{
		const ObjectRef& objRef = this->getValue();
		std::ostringstream res;
		res << objRef;
		return res.str();
	}

	/// Avoid to be considered like object link in parsing
	bool canBeLinked() const
	{ 
		return false; 
	}
};

class SOFA_CORE_API DataVectorObjectRef : public sofa::core::objectmodel::Data< VectorObjectRef >
{
	typedef sofa::core::objectmodel::Data< VectorObjectRef > Inherit;

public:
	/** Constructor
        this constructor should be used through the initData() methods
     */
    explicit DataVectorObjectRef(const BaseData::BaseInitData& init)
    : Inherit(init)
    {
    }

    /** Constructor
        this constructor should be used through the initData() methods
     */
    explicit DataVectorObjectRef(const Inherit::InitData& init)
    : Inherit(init)
    {
    }

	//add a new reference
	void setValue(const std::string& v)
    {
		VectorObjectRef& objRefs = *this->beginEdit();
		ObjectRef objRef;
		if(objRef.setPath(v))
			objRefs.push_back(objRef);
		else
			this->getOwner()->serr << v << "is not a correct link (Have you forgotten the @ ?)" << this->getOwner()->sendl;
		this->endEdit();
    }

	//replace list of references
	void setValue(const helper::vector<std::string>& v)
    {
		VectorObjectRef& objRefs = *this->beginEdit();
		objRefs.clear();
		for(unsigned int i=0 ; i<v.size() ; i++)
		{
			ObjectRef objRef;
			if(objRef.setPath(v[i]))
				objRefs.push_back(objRef);
			else
				this->getOwner()->serr << v[i] << "is not a correct link (Have you forgotten the @ ?)" << this->getOwner()->sendl;
		}

		this->endEdit();
    }

	std::string getValueString() const
	{
		const VectorObjectRef& objRefs = this->getValue();
		std::ostringstream res;
		res << objRefs;
		return res.str();
	}

	/// Avoid to be considered like object link in parsing
	bool canBeLinked() const
	{ 
		return false; 
	}

};

} //namespace objectmodel

} //namespace core

} //namespace sofa

#if defined(WIN32) && !defined(SOFA_BUILD_CORE)

extern template class SOFA_CORE_API sofa::core::objectmodel::TData< sofa::core::objectmodel::ObjectRef >;
extern template class SOFA_CORE_API sofa::core::objectmodel::TData< sofa::core::objectmodel::VectorObjectRef >;

#endif // defined(WIN32) && !defined(SOFA_BUILD_CORE)

#endif // SOFA_CORE_OBJECTMODEL_OBJECTREF_H
