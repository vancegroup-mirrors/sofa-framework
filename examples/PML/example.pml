<!-- physical model is a generic representation for 3D physical model (FEM, spring mass network, phymulob...) --> 

<!-- This an example of a pml designed for Sofa -->

<physicalModel name="bodyChain" nrOfAtoms="32"
 nrOfExclusiveComponents="6"
 nrOfInformativeComponents="0"
 nrOfCells="25"
>
<!-- list of atoms: -->
<atoms>
<structuralComponent  name="element list" >
<nrOfStructures value="32"/>
<atom>
<atomProperties index="0" x="0" y="0" z="0" />
</atom>
<atom>
<atomProperties index="1" x="0" y="0" z="1" />
</atom>
<atom>
<atomProperties index="2" x="0" y="1" z="1" />
</atom>
<atom>
<atomProperties index="3" x="0" y="1" z="0" />
</atom>
<atom>
<atomProperties index="4" x="1" y="0" z="0" />
</atom>
<atom>
<atomProperties index="5" x="1" y="0" z="1" />
</atom>
<atom>
<atomProperties index="6" x="1" y="1" z="1" />
</atom>
<atom>
<atomProperties index="7" x="1" y="1" z="0" />
</atom>
<atom>
<atomProperties index="8" x="2" y="0" z="0" />
</atom>
<atom>
<atomProperties index="9" x="2" y="0" z="1" />
</atom>
<atom>
<atomProperties index="10" x="2" y="1" z="1" />
</atom>
<atom>
<atomProperties index="11" x="2" y="1" z="0" />
</atom>
<atom>
<atomProperties index="12" x="3" y="0" z="0" />
</atom>
<atom>
<atomProperties index="13" x="3" y="0" z="1" />
</atom>
<atom>
<atomProperties index="14" x="3" y="1" z="1" />
</atom>
<atom>
<atomProperties index="15" x="3" y="1" z="0" />
</atom>
<atom>
<atomProperties index="16" x="4" y="0" z="0" />
</atom>
<atom>
<atomProperties index="17" x="4" y="0" z="1" />
</atom>
<atom>
<atomProperties index="18" x="4" y="1" z="1" />
</atom>
<atom>
<atomProperties index="19" x="4" y="1" z="0" />
</atom>
<atom>
<atomProperties index="20" x="5" y="0" z="0" />
</atom>
<atom>
<atomProperties index="21" x="5" y="0" z="1" />
</atom>
<atom>
<atomProperties index="22" x="5" y="1" z="1" />
</atom>
<atom>
<atomProperties index="23" x="5" y="1" z="0" />
</atom>
<atom>
<atomProperties index="24" x="4" y="-1" z="0" />
</atom>
<atom>
<atomProperties index="25" x="4" y="-1" z="1" />
</atom>
<atom>
<atomProperties index="26" x="4" y="-2" z="1" />
</atom>
<atom>
<atomProperties index="27" x="4" y="-2" z="0" />
</atom>
<atom>
<atomProperties index="28" x="5" y="-1" z="0" />
</atom>
<atom>
<atomProperties index="29" x="5" y="-1" z="1" />
</atom>
<atom>
<atomProperties index="30" x="5" y="-2" z="1" />
</atom>
<atom>
<atomProperties index="31" x="5" y="-2" z="0" />
</atom>
</structuralComponent>
</atoms>

<!-- list of exclusive components : -->
<exclusiveComponents>
<multiComponent name="Exclusive Components " >
<multiComponent name="Bodies">

<structuralComponent name="CubeFEM" bodyType="FEM" mass="0.1" young="2000"  poisson="0.49"  deformation="LARGE"  collision="true" mode="WIREFRAME_AND_SURFACE">
<color r="0.5" g="0.5" b="0.8" a="1" />
    <cell>
        <cellProperties type="HEXAHEDRON"/>
        <atomRef index="0"/>
        <atomRef index="1"/>
        <atomRef index="2"/>
        <atomRef index="3"/>
        <atomRef index="4"/>
        <atomRef index="5"/>
        <atomRef index="6"/>
        <atomRef index="7"/>
    </cell>
</structuralComponent>
<structuralComponent name="CubeSprings" bodyType="stiffSpring" mass="0.2" stiffness="800" damping="2" collision="true" mode="WIREFRAME_AND_SURFACE">
<color r="0.5" g="0.8" b="0.5" a="1" />
    <cell>
	  <cellProperties type="HEXAHEDRON"/>
        <atomRef index="8"/>
        <atomRef index="9"/>
        <atomRef index="10"/>
        <atomRef index="11"/>
        <atomRef index="12"/>
        <atomRef index="13"/>
        <atomRef index="14"/>
        <atomRef index="15"/>
    </cell>
</structuralComponent>
<structuralComponent name="CubeFEM2" bodyType="FEM" mass="0.2" young="1000"  poisson="0.49"  deformation="LARGE"  collision="true" mode="WIREFRAME_AND_SURFACE">
<color r="0.5" g="0.5" b="0.8" a="1" />
    <cell>
	  <cellProperties type="HEXAHEDRON"/>
        <atomRef index="16"/>
        <atomRef index="17"/>
        <atomRef index="18"/>
        <atomRef index="19"/>
        <atomRef index="20"/>
        <atomRef index="21"/>
        <atomRef index="22"/>
        <atomRef index="23"/>
    </cell>
</structuralComponent>
<structuralComponent name="CubeRigid" bodyType="rigid" mass="1" collision="true" mode="WIREFRAME_AND_SURFACE">
<color r="0.8" g="0.5" b="0.5" a="1" />
    <cell>
        <cellProperties type="QUAD"/>
        <atomRef index="24"/>
        <atomRef index="25"/>
        <atomRef index="26"/>
        <atomRef index="27"/>
    </cell>
    <cell>
        <cellProperties type="QUAD"/>
        <atomRef index="30"/>
        <atomRef index="31"/>
        <atomRef index="27"/>
        <atomRef index="26"/>
    </cell>
    <cell>
        <cellProperties type="QUAD"/>
        <atomRef index="31"/>
        <atomRef index="30"/>
        <atomRef index="29"/>
        <atomRef index="28"/>
    </cell>
    <cell>
        <cellProperties type="QUAD"/>
        <atomRef index="25"/>
        <atomRef index="24"/>
        <atomRef index="28"/>
        <atomRef index="29"/>
    </cell>
    <cell>
        <cellProperties type="QUAD"/>
        <atomRef index="24"/>
        <atomRef index="27"/>
        <atomRef index="31"/>
        <atomRef index="28"/>
    </cell>
    <cell>
        <cellProperties type="QUAD"/>
        <atomRef index="29"/>
        <atomRef index="30"/>
        <atomRef index="26"/>
        <atomRef index="25"/>
    </cell>
</structuralComponent>
<structuralComponent name="Interaction1" bodyType="interaction" stiffness="200" body1="CubeFEM" body2="CubeSprings" mode="WIREFRAME_AND_SURFACE">
<color r="0.5" g="0.5" b="0.5" a="1" />
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="4"/>
        <atomRef index="8"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="5"/>
        <atomRef index="9"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="6"/>
        <atomRef index="10"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="7"/>
        <atomRef index="11"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="4"/>
        <atomRef index="10"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="5"/>
        <atomRef index="11"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="6"/>
        <atomRef index="8"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="7"/>
        <atomRef index="9"/>
    </cell>
</structuralComponent>
<structuralComponent name="Interaction2" bodyType="interaction" stiffness="200" body1="CubeSprings" body2="CubeFEM2" mode="WIREFRAME_AND_SURFACE">
<color r="0.5" g="0.5" b="0.5" a="1" />
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="12"/>
        <atomRef index="16"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="13"/>
        <atomRef index="17"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="14"/>
        <atomRef index="18"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="15"/>
        <atomRef index="19"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="12"/>
        <atomRef index="18"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="13"/>
        <atomRef index="19"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="14"/>
        <atomRef index="16"/>
    </cell>
    <cell>
        <cellProperties type="LINE"/>
        <atomRef index="15"/>
        <atomRef index="17"/>
    </cell>
</structuralComponent>


</multiComponent>
</multiComponent>
</exclusiveComponents>
<!-- list of informative components : -->
<informativeComponents>
</informativeComponents>
</physicalModel>

