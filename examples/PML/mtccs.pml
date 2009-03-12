<!-- physical model is a generic representation for 3D physical model (FEM, spring mass network, phymulob...) --> 

<!-- This an example of a pml designed for Sofa
	It Create 2 FEM cubes (see bodies components) -->

<physicalModel name="cube convex hull" nrOfAtoms="27"
 nrOfExclusiveComponents="3"
 nrOfInformativeComponents="0"
 nrOfCells="52"
>
<!-- list of atoms: -->
<atoms>
<structuralComponent  name="element list" >
<nrOfStructures value="35"/>
<atom>
<atomProperties index="0" x="-5" y="5" z="-5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="1" x="5" y="5" z="-5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="2" x="5" y="5" z="5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="3" x="-5" y="5" z="5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="4" x="-5" y="-5" z="-5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="5" x="5" y="-5" z="-5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="6" x="5" y="-5" z="5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="7" x="-5" y="-5" z="5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="10" x="0" y="5" z="-5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="21" x="5" y="5" z="0"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="30" x="-5" y="5" z="0"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="32" x="0" y="5" z="5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="40" x="-5" y="0" z="-5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="51" x="5" y="0" z="-5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="54" x="0" y="-5" z="-5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="62" x="5" y="0" z="5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="65" x="5" y="-5" z="0"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="73" x="-5" y="0" z="5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="74" x="-5" y="-5" z="0"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="76" x="0" y="-5" z="5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="100" x="0" y="0" z="0"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="101" x="0" y="5" z="0"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="102" x="0" y="0" z="-5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="103" x="5" y="0" z="0"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="104" x="0" y="0" z="5"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="105" x="-5" y="0" z="0"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="106" x="0" y="-5" z="0"   mass="29.6296" />
</atom>
<atom>
<atomProperties index="200" x="-10" y="-10" z="-10"/>
</atom>
<atom>
<atomProperties index="201" x="10" y="-10" z="-10"/>
</atom>
<atom>
<atomProperties index="202" x="10" y="-10" z="10"/>
</atom>
<atom>
<atomProperties index="203" x="-10" y="-10" z="10"/>
</atom>
<atom>
<atomProperties index="204" x="-10" y="-12" z="-10"/>
</atom>
<atom>
<atomProperties index="205" x="10" y="-12" z="-10"/>
</atom>
<atom>
<atomProperties index="206" x="10" y="-12" z="10"/>
</atom>
<atom>
<atomProperties index="207" x="-10" y="-12" z="10"/>
</atom>
</structuralComponent>
</atoms>
<!-- list of exclusive components : -->
<exclusiveComponents>
<multiComponent name="Exclusive Components " >
<multiComponent name="Bodies">
<structuralComponent name="Cube" bodyType="FEM" mass="0.1" young="100"  poisson="0.49"  deformation="LARGE"  collision="true" mode="WIREFRAME_AND_SURFACE">
<color r="0.5" g="0.5" b="0.8" a="1" />
    <cell>
        <cellProperties type="HEXAHEDRON"/>
        <atomRef index="104"/>
        <atomRef index="73"/>
        <atomRef index="3"/>
        <atomRef index="32"/>
        <atomRef index="100"/>
        <atomRef index="105"/>
        <atomRef index="30"/>
        <atomRef index="101"/>
    </cell>
    <cell>
        <cellProperties type="HEXAHEDRON"/>
        <atomRef index="100"/>
        <atomRef index="105"/>
        <atomRef index="30"/>
        <atomRef index="101"/>
        <atomRef index="102"/>
        <atomRef index="40"/>
        <atomRef index="0"/>
        <atomRef index="10"/>
    </cell>
    <cell>
        <cellProperties type="HEXAHEDRON"/>
        <atomRef index="62"/>
        <atomRef index="104"/>
        <atomRef index="32"/>
        <atomRef index="2"/>
        <atomRef index="103"/>
        <atomRef index="100"/>
        <atomRef index="101"/>
        <atomRef index="21"/>
    </cell>
    <cell>
        <cellProperties type="HEXAHEDRON"/>
        <atomRef index="103"/>
        <atomRef index="100"/>
        <atomRef index="101"/>
        <atomRef index="21"/>
        <atomRef index="51"/>
        <atomRef index="102"/>
        <atomRef index="10"/>
        <atomRef index="1"/>
    </cell>
    <cell>
        <cellProperties type="HEXAHEDRON"/>
        <atomRef index="76"/>
        <atomRef index="7"/>
        <atomRef index="73"/>
        <atomRef index="104"/>
        <atomRef index="106"/>
        <atomRef index="74"/>
        <atomRef index="105"/>
        <atomRef index="100"/>
    </cell>
    <cell>
        <cellProperties type="HEXAHEDRON"/>
        <atomRef index="106"/>
        <atomRef index="74"/>
        <atomRef index="105"/>
        <atomRef index="100"/>
        <atomRef index="54"/>
        <atomRef index="4"/>
        <atomRef index="40"/>
        <atomRef index="102"/>
    </cell>
    <cell>
        <cellProperties type="HEXAHEDRON"/>
        <atomRef index="6"/>
        <atomRef index="76"/>
        <atomRef index="104"/>
        <atomRef index="62"/>
        <atomRef index="65"/>
        <atomRef index="106"/>
        <atomRef index="100"/>
        <atomRef index="103"/>
    </cell>
    <cell>
        <cellProperties type="HEXAHEDRON"/>
        <atomRef index="65"/>
        <atomRef index="106"/>
        <atomRef index="100"/>
        <atomRef index="103"/>
        <atomRef index="5"/>
        <atomRef index="54"/>
        <atomRef index="102"/>
        <atomRef index="51"/>
    </cell>
</structuralComponent>
<structuralComponent name="CubeFloor" bodyType="FEM" mass="0" collision="true" mode="WIREFRAME_AND_SURFACE">
<color r="0.5" g="0.5" b="0.5" a="1" />
    <cell>
        <cellProperties type="HEXAHEDRON"/>
        <atomRef index="200"/>
        <atomRef index="201"/>
        <atomRef index="202"/>
        <atomRef index="203"/>
        <atomRef index="204"/>
        <atomRef index="205"/>
        <atomRef index="206"/>
        <atomRef index="207"/>
    </cell>
</structuralComponent>
</multiComponent>
</multiComponent>
</exclusiveComponents>
<!-- list of informative components : -->
<informativeComponents>
</informativeComponents>
</physicalModel>
