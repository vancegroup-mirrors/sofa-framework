# Target is a library:  colladadom

SOFA_DIR = ../../..
TEMPLATE = lib
include($${SOFA_DIR}/sofa.cfg)

TARGET = colladadom$$LIBSUFFIX
CONFIG += $$CONFIGLIBRARIES

#DEFINES *= DOM_INCLUDE_LIBXML
DEFINES *= DOM_INCLUDE_TINYXML TINYXML NO_ZAE NO_BOOST

LIBS -= -lphysicalmodel$$LIBSUFFIX
LIBS -= -lload$$LIBSUFFIX
LIBS -= -lcolladadom$$LIBSUFFIX

CONFIG += warn_off

INCLUDEPATH *= external-libs/minizip/include external-libs/tinyxml

win32 {
  DEFINES *= DOM_DYNAMIC PCRE_STATIC
  contains (DEFINES, DOM_DYNAMIC) {
    !contains(CONFIGSTATIC, static) {
	CONFIG -= staticlib
    CONFIG += dll
}
    DEFINES *= DOM_EXPORT
  }
  contains (CONFIGDEBUG, debug) {
    PCRESUFFIX = -d
  } else {
    PCRESUFFIX =
  }
  win32-msvc2008{
    BUILDID=vc9
  } else {
    BUILDID=vc8
  }
#  LIBS += external-libs/pcre/lib/$$BUILDID/pcrecpp$$PCRESUFFIX external-libs/pcre/lib/$$BUILDID/pcre$$PCRESUFFIX
  QMAKE_LIBDIR += external-libs/pcre/lib/$$BUILDID external-libs/tinyxml/lib/$$BUILDID
  LIBS += -lpcrecpp$$PCRESUFFIX -lpcre$$PCRESUFFIX
  LIBS += -ltinyxml$$LIBSUFFIX
  INCLUDEPATH *= include include/1.4 external-libs/pcre
}

unix {
  macx {
    LIBS += external-libs/pcre/lib/mac/libpcrecpp.a external-libs/pcre/lib/mac/libpcre.a
    INCLUDEPATH *= include include/1.4 external-libs/pcre
  } 
  else {
    LIBS += -lpcrecpp -lpcre
  }
}

HEADERS += \
           include/1.4/dom/domAccessor.h \
           include/1.4/dom/domAnimation.h \
           include/1.4/dom/domAnimation_clip.h \
           include/1.4/dom/domAsset.h \
           include/1.4/dom/domBind_material.h \
           include/1.4/dom/domBool_array.h \
           include/1.4/dom/domBox.h \
           include/1.4/dom/domCamera.h \
           include/1.4/dom/domCapsule.h \
           include/1.4/dom/domCg_connect_param.h \
           include/1.4/dom/domCg_newarray_type.h \
           include/1.4/dom/domCg_newparam.h \
           include/1.4/dom/domCg_param_type.h \
           include/1.4/dom/domCg_sampler1D.h \
           include/1.4/dom/domCg_sampler2D.h \
           include/1.4/dom/domCg_sampler3D.h \
           include/1.4/dom/domCg_samplerCUBE.h \
           include/1.4/dom/domCg_samplerDEPTH.h \
           include/1.4/dom/domCg_samplerRECT.h \
           include/1.4/dom/domCg_setarray_type.h \
           include/1.4/dom/domCg_setparam.h \
           include/1.4/dom/domCg_setparam_simple.h \
           include/1.4/dom/domCg_setuser_type.h \
           include/1.4/dom/domCg_surface_type.h \
           include/1.4/dom/domChannel.h \
           include/1.4/dom/domCOLLADA.h \
           include/1.4/dom/domCommon_color_or_texture_type.h \
           include/1.4/dom/domCommon_float_or_param_type.h \
           include/1.4/dom/domCommon_newparam_type.h \
           include/1.4/dom/domCommon_transparent_type.h \
           include/1.4/dom/domConstants.h \
           include/1.4/dom/domController.h \
           include/1.4/dom/domConvex_mesh.h \
           include/1.4/dom/domCylinder.h \
           include/1.4/dom/domEffect.h \
           include/1.4/dom/domElements.h \
           include/1.4/dom/domEllipsoid.h \
           include/1.4/dom/domExtra.h \
           include/1.4/dom/domFloat_array.h \
           include/1.4/dom/domForce_field.h \
           include/1.4/dom/domFx_annotate_common.h \
           include/1.4/dom/domFx_annotate_type_common.h \
           include/1.4/dom/domFx_basic_type_common.h \
           include/1.4/dom/domFx_clearcolor_common.h \
           include/1.4/dom/domFx_cleardepth_common.h \
           include/1.4/dom/domFx_clearstencil_common.h \
           include/1.4/dom/domFx_code_profile.h \
           include/1.4/dom/domFx_colortarget_common.h \
           include/1.4/dom/domFx_depthtarget_common.h \
           include/1.4/dom/domFx_include_common.h \
           include/1.4/dom/domFx_newparam_common.h \
           include/1.4/dom/domFx_profile_abstract.h \
           include/1.4/dom/domFx_sampler1D_common.h \
           include/1.4/dom/domFx_sampler2D_common.h \
           include/1.4/dom/domFx_sampler3D_common.h \
           include/1.4/dom/domFx_samplerCUBE_common.h \
           include/1.4/dom/domFx_samplerDEPTH_common.h \
           include/1.4/dom/domFx_samplerRECT_common.h \
           include/1.4/dom/domFx_stenciltarget_common.h \
           include/1.4/dom/domFx_surface_common.h \
           include/1.4/dom/domFx_surface_format_hint_common.h \
           include/1.4/dom/domFx_surface_init_common.h \
           include/1.4/dom/domFx_surface_init_cube_common.h \
           include/1.4/dom/domFx_surface_init_from_common.h \
           include/1.4/dom/domFx_surface_init_planar_common.h \
           include/1.4/dom/domFx_surface_init_volume_common.h \
           include/1.4/dom/domGeometry.h \
           include/1.4/dom/domGl_hook_abstract.h \
           include/1.4/dom/domGl_pipeline_settings.h \
           include/1.4/dom/domGl_sampler1D.h \
           include/1.4/dom/domGl_sampler2D.h \
           include/1.4/dom/domGl_sampler3D.h \
           include/1.4/dom/domGl_samplerCUBE.h \
           include/1.4/dom/domGl_samplerDEPTH.h \
           include/1.4/dom/domGl_samplerRECT.h \
           include/1.4/dom/domGles_basic_type_common.h \
           include/1.4/dom/domGles_newparam.h \
           include/1.4/dom/domGles_pipeline_settings.h \
           include/1.4/dom/domGles_sampler_state.h \
           include/1.4/dom/domGles_texcombiner_argumentAlpha_type.h \
           include/1.4/dom/domGles_texcombiner_argumentRGB_type.h \
           include/1.4/dom/domGles_texcombiner_command_type.h \
           include/1.4/dom/domGles_texcombiner_commandAlpha_type.h \
           include/1.4/dom/domGles_texcombiner_commandRGB_type.h \
           include/1.4/dom/domGles_texenv_command_type.h \
           include/1.4/dom/domGles_texture_constant_type.h \
           include/1.4/dom/domGles_texture_pipeline.h \
           include/1.4/dom/domGles_texture_unit.h \
           include/1.4/dom/domGlsl_newarray_type.h \
           include/1.4/dom/domGlsl_newparam.h \
           include/1.4/dom/domGlsl_param_type.h \
           include/1.4/dom/domGlsl_setarray_type.h \
           include/1.4/dom/domGlsl_setparam.h \
           include/1.4/dom/domGlsl_setparam_simple.h \
           include/1.4/dom/domGlsl_surface_type.h \
           include/1.4/dom/domIDREF_array.h \
           include/1.4/dom/domImage.h \
           include/1.4/dom/domInputGlobal.h \
           include/1.4/dom/domInputLocal.h \
           include/1.4/dom/domInputLocalOffset.h \
           include/1.4/dom/domInstance_camera.h \
           include/1.4/dom/domInstance_controller.h \
           include/1.4/dom/domInstance_effect.h \
           include/1.4/dom/domInstance_force_field.h \
           include/1.4/dom/domInstance_geometry.h \
           include/1.4/dom/domInstance_light.h \
           include/1.4/dom/domInstance_material.h \
           include/1.4/dom/domInstance_node.h \
           include/1.4/dom/domInstance_physics_material.h \
           include/1.4/dom/domInstance_physics_model.h \
           include/1.4/dom/domInstance_rigid_body.h \
           include/1.4/dom/domInstance_rigid_constraint.h \
           include/1.4/dom/domInstanceWithExtra.h \
           include/1.4/dom/domInt_array.h \
           include/1.4/dom/domLibrary_animation_clips.h \
           include/1.4/dom/domLibrary_animations.h \
           include/1.4/dom/domLibrary_cameras.h \
           include/1.4/dom/domLibrary_controllers.h \
           include/1.4/dom/domLibrary_effects.h \
           include/1.4/dom/domLibrary_force_fields.h \
           include/1.4/dom/domLibrary_geometries.h \
           include/1.4/dom/domLibrary_images.h \
           include/1.4/dom/domLibrary_lights.h \
           include/1.4/dom/domLibrary_materials.h \
           include/1.4/dom/domLibrary_nodes.h \
           include/1.4/dom/domLibrary_physics_materials.h \
           include/1.4/dom/domLibrary_physics_models.h \
           include/1.4/dom/domLibrary_physics_scenes.h \
           include/1.4/dom/domLibrary_visual_scenes.h \
           include/1.4/dom/domLight.h \
           include/1.4/dom/domLines.h \
           include/1.4/dom/domLinestrips.h \
           include/1.4/dom/domLookat.h \
           include/1.4/dom/domMaterial.h \
           include/1.4/dom/domMatrix.h \
           include/1.4/dom/domMesh.h \
           include/1.4/dom/domMorph.h \
           include/1.4/dom/domName_array.h \
           include/1.4/dom/domNode.h \
           include/1.4/dom/domP.h \
           include/1.4/dom/domParam.h \
           include/1.4/dom/domPhysics_material.h \
           include/1.4/dom/domPhysics_model.h \
           include/1.4/dom/domPhysics_scene.h \
           include/1.4/dom/domPlane.h \
           include/1.4/dom/domPolygons.h \
           include/1.4/dom/domPolylist.h \
           include/1.4/dom/domProfile_CG.h \
           include/1.4/dom/domProfile_COMMON.h \
           include/1.4/dom/domProfile_GLES.h \
           include/1.4/dom/domProfile_GLSL.h \
           include/1.4/dom/domRigid_body.h \
           include/1.4/dom/domRigid_constraint.h \
           include/1.4/dom/domRotate.h \
           include/1.4/dom/domSampler.h \
           include/1.4/dom/domScale.h \
           include/1.4/dom/domSkew.h \
           include/1.4/dom/domSkin.h \
           include/1.4/dom/domSource.h \
           include/1.4/dom/domSphere.h \
           include/1.4/dom/domSpline.h \
           include/1.4/dom/domTapered_capsule.h \
           include/1.4/dom/domTapered_cylinder.h \
           include/1.4/dom/domTargetableFloat.h \
           include/1.4/dom/domTargetableFloat3.h \
           include/1.4/dom/domTechnique.h \
           include/1.4/dom/domTranslate.h \
           include/1.4/dom/domTriangles.h \
           include/1.4/dom/domTrifans.h \
           include/1.4/dom/domTristrips.h \
           include/1.4/dom/domTypes.h \
           include/1.4/dom/domVertices.h \
           include/1.4/dom/domVisual_scene.h \
           include/dae/daeArray.h \
           include/dae/daeArrayTypes.h \
           include/dae/daeAtomicType.h \
           include/dae/daeDatabase.h \
           include/dae/daeDocument.h \
           include/dae/daeDom.h \
           include/dae/daeDomTypes.h \
           include/dae/daeElement.h \
           include/dae/daeError.h \
           include/dae/daeErrorHandler.h \
           include/dae/daeGCCPlatform.h \
           include/dae/daeIDRef.h \
           include/dae/daeIOPlugin.h \
           include/dae/daeIOPluginCommon.h \
           include/dae/daeMemorySystem.h \
           include/dae/daeMetaAny.h \
           include/dae/daeMetaAttribute.h \
           include/dae/daeMetaChoice.h \
           include/dae/daeMetaCMPolicy.h \
           include/dae/daeMetaElement.h \
           include/dae/daeMetaElementAttribute.h \
           include/dae/daeMetaGroup.h \
           include/dae/daeMetaSequence.h \
           include/dae/daePlatform.h \
           include/dae/daeRawResolver.h \
           include/dae/daeRefCountedObj.h \
           include/dae/daeSIDResolver.h \
           include/dae/daeSmartRef.h \
           include/dae/daeStandardURIResolver.h \
           include/dae/daeStringRef.h \
           include/dae/daeStringTable.h \
           include/dae/daeTinyXMLPlugin.h \
           include/dae/daeTypes.h \
           include/dae/daeURI.h \
           include/dae/daeUtils.h \
           include/dae/daeWin32Platform.h \
           include/dae/domAny.h \
           include/dae.h \
           include/dom.h \
#           include/modules/daeLIBXMLPlugin.h \
           include/modules/daeSTLDatabase.h \
           include/modules/stdErrPlugin.h

SOURCES += \
           src/dae/dae.cpp \
           src/dae/daeArray.cpp \
           src/dae/daeAtomicType.cpp \
           src/dae/daeDatabase.cpp \
           src/dae/daeDocument.cpp \
           src/dae/daeDom.cpp \
           src/dae/daeElement.cpp \
           src/dae/daeError.cpp \
           src/dae/daeErrorHandler.cpp \
           src/dae/daeIDRef.cpp \
           src/dae/daeIOPluginCommon.cpp \
           src/dae/daeMemorySystem.cpp \
           src/dae/daeMetaAny.cpp \
           src/dae/daeMetaAttribute.cpp \
           src/dae/daeMetaCMPolicy.cpp \
           src/dae/daeMetaChoice.cpp \
           src/dae/daeMetaElement.cpp \
           src/dae/daeMetaElementAttribute.cpp \
           src/dae/daeMetaGroup.cpp \
           src/dae/daeMetaSequence.cpp \
           src/dae/daeRawResolver.cpp \
           src/dae/daeRefCountedObj.cpp \
           src/dae/daeSIDResolver.cpp \
           src/dae/daeStandardURIResolver.cpp \
           src/dae/daeStringRef.cpp \
           src/dae/daeStringTable.cpp \
           src/dae/daeTinyXMLPlugin.cpp \
           src/dae/daeURI.cpp \
           src/dae/daeUtils.cpp \
           src/dae/domAny.cpp \
           src/modules/stdErrPlugin/stdErrPlugin.cpp \
           src/modules/STLDatabase/daeSTLDatabase.cpp \
#           src/modules/LIBXMLPlugin/daeLIBXMLPlugin.cpp \
           src/1.4/dom/domAccessor.cpp \
           src/1.4/dom/domAnimation.cpp \
           src/1.4/dom/domAnimation_clip.cpp \
           src/1.4/dom/domAsset.cpp \
           src/1.4/dom/domBind_material.cpp \
           src/1.4/dom/domBool_array.cpp \
           src/1.4/dom/domBox.cpp \
           src/1.4/dom/domCOLLADA.cpp \
           src/1.4/dom/domCamera.cpp \
           src/1.4/dom/domCapsule.cpp \
           src/1.4/dom/domCg_connect_param.cpp \
           src/1.4/dom/domCg_newarray_type.cpp \
           src/1.4/dom/domCg_newparam.cpp \
           src/1.4/dom/domCg_param_type.cpp \
           src/1.4/dom/domCg_sampler1D.cpp \
           src/1.4/dom/domCg_sampler2D.cpp \
           src/1.4/dom/domCg_sampler3D.cpp \
           src/1.4/dom/domCg_samplerCUBE.cpp \
           src/1.4/dom/domCg_samplerDEPTH.cpp \
           src/1.4/dom/domCg_samplerRECT.cpp \
           src/1.4/dom/domCg_setarray_type.cpp \
           src/1.4/dom/domCg_setparam.cpp \
           src/1.4/dom/domCg_setparam_simple.cpp \
           src/1.4/dom/domCg_setuser_type.cpp \
           src/1.4/dom/domCg_surface_type.cpp \
           src/1.4/dom/domChannel.cpp \
           src/1.4/dom/domCommon_color_or_texture_type.cpp \
           src/1.4/dom/domCommon_float_or_param_type.cpp \
           src/1.4/dom/domCommon_newparam_type.cpp \
           src/1.4/dom/domCommon_transparent_type.cpp \
           src/1.4/dom/domConstants.cpp \
           src/1.4/dom/domController.cpp \
           src/1.4/dom/domConvex_mesh.cpp \
           src/1.4/dom/domCylinder.cpp \
           src/1.4/dom/domEffect.cpp \
           src/1.4/dom/domEllipsoid.cpp \
           src/1.4/dom/domExtra.cpp \
           src/1.4/dom/domFloat_array.cpp \
           src/1.4/dom/domForce_field.cpp \
           src/1.4/dom/domFx_annotate_common.cpp \
           src/1.4/dom/domFx_annotate_type_common.cpp \
           src/1.4/dom/domFx_basic_type_common.cpp \
           src/1.4/dom/domFx_clearcolor_common.cpp \
           src/1.4/dom/domFx_cleardepth_common.cpp \
           src/1.4/dom/domFx_clearstencil_common.cpp \
           src/1.4/dom/domFx_code_profile.cpp \
           src/1.4/dom/domFx_colortarget_common.cpp \
           src/1.4/dom/domFx_depthtarget_common.cpp \
           src/1.4/dom/domFx_include_common.cpp \
           src/1.4/dom/domFx_newparam_common.cpp \
           src/1.4/dom/domFx_profile_abstract.cpp \
           src/1.4/dom/domFx_sampler1D_common.cpp \
           src/1.4/dom/domFx_sampler2D_common.cpp \
           src/1.4/dom/domFx_sampler3D_common.cpp \
           src/1.4/dom/domFx_samplerCUBE_common.cpp \
           src/1.4/dom/domFx_samplerDEPTH_common.cpp \
           src/1.4/dom/domFx_samplerRECT_common.cpp \
           src/1.4/dom/domFx_stenciltarget_common.cpp \
           src/1.4/dom/domFx_surface_common.cpp \
           src/1.4/dom/domFx_surface_format_hint_common.cpp \
           src/1.4/dom/domFx_surface_init_common.cpp \
           src/1.4/dom/domFx_surface_init_cube_common.cpp \
           src/1.4/dom/domFx_surface_init_from_common.cpp \
           src/1.4/dom/domFx_surface_init_planar_common.cpp \
           src/1.4/dom/domFx_surface_init_volume_common.cpp \
           src/1.4/dom/domGeometry.cpp \
           src/1.4/dom/domGl_hook_abstract.cpp \
           src/1.4/dom/domGl_pipeline_settings.cpp \
           src/1.4/dom/domGl_sampler1D.cpp \
           src/1.4/dom/domGl_sampler2D.cpp \
           src/1.4/dom/domGl_sampler3D.cpp \
           src/1.4/dom/domGl_samplerCUBE.cpp \
           src/1.4/dom/domGl_samplerDEPTH.cpp \
           src/1.4/dom/domGl_samplerRECT.cpp \
           src/1.4/dom/domGles_basic_type_common.cpp \
           src/1.4/dom/domGles_newparam.cpp \
           src/1.4/dom/domGles_pipeline_settings.cpp \
           src/1.4/dom/domGles_sampler_state.cpp \
           src/1.4/dom/domGles_texcombiner_argumentAlpha_type.cpp \
           src/1.4/dom/domGles_texcombiner_argumentRGB_type.cpp \
           src/1.4/dom/domGles_texcombiner_commandAlpha_type.cpp \
           src/1.4/dom/domGles_texcombiner_commandRGB_type.cpp \
           src/1.4/dom/domGles_texcombiner_command_type.cpp \
           src/1.4/dom/domGles_texenv_command_type.cpp \
           src/1.4/dom/domGles_texture_constant_type.cpp \
           src/1.4/dom/domGles_texture_pipeline.cpp \
           src/1.4/dom/domGles_texture_unit.cpp \
           src/1.4/dom/domGlsl_newarray_type.cpp \
           src/1.4/dom/domGlsl_newparam.cpp \
           src/1.4/dom/domGlsl_param_type.cpp \
           src/1.4/dom/domGlsl_setarray_type.cpp \
           src/1.4/dom/domGlsl_setparam.cpp \
           src/1.4/dom/domGlsl_setparam_simple.cpp \
           src/1.4/dom/domGlsl_surface_type.cpp \
           src/1.4/dom/domIDREF_array.cpp \
           src/1.4/dom/domImage.cpp \
           src/1.4/dom/domInputGlobal.cpp \
           src/1.4/dom/domInputLocal.cpp \
           src/1.4/dom/domInputLocalOffset.cpp \
           src/1.4/dom/domInstanceWithExtra.cpp \
           src/1.4/dom/domInstance_camera.cpp \
           src/1.4/dom/domInstance_controller.cpp \
           src/1.4/dom/domInstance_effect.cpp \
           src/1.4/dom/domInstance_force_field.cpp \
           src/1.4/dom/domInstance_geometry.cpp \
           src/1.4/dom/domInstance_light.cpp \
           src/1.4/dom/domInstance_material.cpp \
           src/1.4/dom/domInstance_node.cpp \
           src/1.4/dom/domInstance_physics_material.cpp \
           src/1.4/dom/domInstance_physics_model.cpp \
           src/1.4/dom/domInstance_rigid_body.cpp \
           src/1.4/dom/domInstance_rigid_constraint.cpp \
           src/1.4/dom/domInt_array.cpp \
           src/1.4/dom/domLibrary_animation_clips.cpp \
           src/1.4/dom/domLibrary_animations.cpp \
           src/1.4/dom/domLibrary_cameras.cpp \
           src/1.4/dom/domLibrary_controllers.cpp \
           src/1.4/dom/domLibrary_effects.cpp \
           src/1.4/dom/domLibrary_force_fields.cpp \
           src/1.4/dom/domLibrary_geometries.cpp \
           src/1.4/dom/domLibrary_images.cpp \
           src/1.4/dom/domLibrary_lights.cpp \
           src/1.4/dom/domLibrary_materials.cpp \
           src/1.4/dom/domLibrary_nodes.cpp \
           src/1.4/dom/domLibrary_physics_materials.cpp \
           src/1.4/dom/domLibrary_physics_models.cpp \
           src/1.4/dom/domLibrary_physics_scenes.cpp \
           src/1.4/dom/domLibrary_visual_scenes.cpp \
           src/1.4/dom/domLight.cpp \
           src/1.4/dom/domLines.cpp \
           src/1.4/dom/domLinestrips.cpp \
           src/1.4/dom/domLookat.cpp \
           src/1.4/dom/domMaterial.cpp \
           src/1.4/dom/domMatrix.cpp \
           src/1.4/dom/domMesh.cpp \
           src/1.4/dom/domMorph.cpp \
           src/1.4/dom/domName_array.cpp \
           src/1.4/dom/domNode.cpp \
           src/1.4/dom/domP.cpp \
           src/1.4/dom/domParam.cpp \
           src/1.4/dom/domPhysics_material.cpp \
           src/1.4/dom/domPhysics_model.cpp \
           src/1.4/dom/domPhysics_scene.cpp \
           src/1.4/dom/domPlane.cpp \
           src/1.4/dom/domPolygons.cpp \
           src/1.4/dom/domPolylist.cpp \
           src/1.4/dom/domProfile_CG.cpp \
           src/1.4/dom/domProfile_COMMON.cpp \
           src/1.4/dom/domProfile_GLES.cpp \
           src/1.4/dom/domProfile_GLSL.cpp \
           src/1.4/dom/domRigid_body.cpp \
           src/1.4/dom/domRigid_constraint.cpp \
           src/1.4/dom/domRotate.cpp \
           src/1.4/dom/domSampler.cpp \
           src/1.4/dom/domScale.cpp \
           src/1.4/dom/domSkew.cpp \
           src/1.4/dom/domSkin.cpp \
           src/1.4/dom/domSource.cpp \
           src/1.4/dom/domSphere.cpp \
           src/1.4/dom/domSpline.cpp \
           src/1.4/dom/domTapered_capsule.cpp \
           src/1.4/dom/domTapered_cylinder.cpp \
           src/1.4/dom/domTargetableFloat.cpp \
           src/1.4/dom/domTargetableFloat3.cpp \
           src/1.4/dom/domTechnique.cpp \
           src/1.4/dom/domTranslate.cpp \
           src/1.4/dom/domTriangles.cpp \
           src/1.4/dom/domTrifans.cpp \
           src/1.4/dom/domTristrips.cpp \
           src/1.4/dom/domTypes.cpp \
           src/1.4/dom/domVertices.cpp \
           src/1.4/dom/domVisual_scene.cpp
