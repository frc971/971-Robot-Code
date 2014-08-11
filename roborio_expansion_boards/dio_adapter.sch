<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="6.5.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="custom">
<packages>
<package name="22-23-2031">
<description>3-pin Molex locking connector (male) 22-23-2031.</description>
<pad name="2" x="0" y="0" drill="1.016"/>
<pad name="1" x="0" y="2.54" drill="1.016"/>
<pad name="3" x="0" y="-2.54" drill="1.016"/>
<wire x1="-3.1242" y1="3.7338" x2="1.0668" y2="3.7338" width="0.127" layer="21"/>
<wire x1="1.0668" y1="3.7338" x2="1.0668" y2="-3.7338" width="0.127" layer="21"/>
<wire x1="1.0668" y1="-3.7338" x2="-3.1242" y2="-3.7338" width="0.127" layer="21"/>
<wire x1="-3.1242" y1="-3.7338" x2="-3.1242" y2="3.7338" width="0.127" layer="21"/>
<rectangle x1="-3.1242" y1="-3.7338" x2="3.0988" y2="3.7338" layer="39"/>
<wire x1="3.0988" y1="3.7338" x2="3.0988" y2="-3.7338" width="0.127" layer="21"/>
<wire x1="3.0988" y1="3.7338" x2="1.0668" y2="3.7338" width="0.127" layer="21"/>
<wire x1="3.0988" y1="-3.7338" x2="1.0668" y2="-3.7338" width="0.127" layer="21"/>
</package>
<package name="PWM_1X3">
<description>Generic 1x3 0.1" pitch connector.</description>
<pad name="2" x="0" y="0" drill="1.016"/>
<pad name="1" x="0" y="2.54" drill="1.016"/>
<pad name="3" x="0" y="-2.54" drill="1.016"/>
<wire x1="-1.27" y1="3.81" x2="1.27" y2="3.81" width="0.127" layer="21"/>
<wire x1="1.27" y1="3.81" x2="1.27" y2="-3.81" width="0.127" layer="21"/>
<wire x1="1.27" y1="-3.81" x2="-1.27" y2="-3.81" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-3.81" x2="-1.27" y2="3.81" width="0.127" layer="21"/>
<rectangle x1="-1.27" y1="-3.81" x2="1.27" y2="3.81" layer="39"/>
</package>
</packages>
<symbols>
<symbol name="22-23-2031">
<description>3-pin Molex locking connector (male).</description>
<pin name="PWR" x="-7.62" y="0" length="middle" direction="pwr"/>
<pin name="GND" x="-7.62" y="5.08" length="middle" direction="pwr"/>
<pin name="SIG" x="-7.62" y="-5.08" length="middle"/>
<wire x1="-2.54" y1="7.62" x2="5.08" y2="7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="-7.62" x2="-2.54" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-7.62" x2="-2.54" y2="7.62" width="0.254" layer="94"/>
<text x="-5.08" y="8.255" size="1.778" layer="95">&gt;NAME</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="22-23-2031">
<description>Molex 3-pin male connector 22-23-2031.</description>
<gates>
<gate name="G$1" symbol="22-23-2031" x="0" y="0"/>
</gates>
<devices>
<device name="" package="22-23-2031">
<connects>
<connect gate="G$1" pin="GND" pad="1"/>
<connect gate="G$1" pin="PWR" pad="2"/>
<connect gate="G$1" pin="SIG" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="GENERIC" package="PWM_1X3">
<connects>
<connect gate="G$1" pin="GND" pad="1"/>
<connect gate="G$1" pin="PWR" pad="2"/>
<connect gate="G$1" pin="SIG" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U$1" library="custom" deviceset="22-23-2031" device="GENERIC"/>
<part name="U$2" library="custom" deviceset="22-23-2031" device="GENERIC"/>
<part name="U$3" library="custom" deviceset="22-23-2031" device="GENERIC"/>
<part name="U$4" library="custom" deviceset="22-23-2031" device="GENERIC"/>
<part name="U$5" library="custom" deviceset="22-23-2031" device="GENERIC"/>
<part name="U$6" library="custom" deviceset="22-23-2031" device="GENERIC"/>
<part name="U$7" library="custom" deviceset="22-23-2031" device="GENERIC"/>
<part name="U$8" library="custom" deviceset="22-23-2031" device="GENERIC"/>
<part name="U$9" library="custom" deviceset="22-23-2031" device="GENERIC"/>
<part name="U$10" library="custom" deviceset="22-23-2031" device="GENERIC"/>
<part name="U$11" library="custom" deviceset="22-23-2031" device=""/>
<part name="U$12" library="custom" deviceset="22-23-2031" device=""/>
<part name="U$13" library="custom" deviceset="22-23-2031" device=""/>
<part name="U$14" library="custom" deviceset="22-23-2031" device=""/>
<part name="U$15" library="custom" deviceset="22-23-2031" device=""/>
<part name="U$16" library="custom" deviceset="22-23-2031" device=""/>
<part name="U$17" library="custom" deviceset="22-23-2031" device=""/>
<part name="U$18" library="custom" deviceset="22-23-2031" device=""/>
<part name="U$19" library="custom" deviceset="22-23-2031" device=""/>
<part name="U$20" library="custom" deviceset="22-23-2031" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="10.16" y="10.16" rot="MR0"/>
<instance part="U$2" gate="G$1" x="10.16" y="30.48" rot="MR0"/>
<instance part="U$3" gate="G$1" x="10.16" y="50.8" rot="MR0"/>
<instance part="U$4" gate="G$1" x="10.16" y="71.12" rot="MR0"/>
<instance part="U$5" gate="G$1" x="10.16" y="91.44" rot="MR0"/>
<instance part="U$6" gate="G$1" x="10.16" y="111.76" rot="MR0"/>
<instance part="U$7" gate="G$1" x="10.16" y="132.08" rot="MR0"/>
<instance part="U$8" gate="G$1" x="10.16" y="152.4" rot="MR0"/>
<instance part="U$9" gate="G$1" x="10.16" y="172.72" rot="MR0"/>
<instance part="U$10" gate="G$1" x="10.16" y="193.04" rot="MR0"/>
<instance part="U$11" gate="G$1" x="33.02" y="10.16"/>
<instance part="U$12" gate="G$1" x="33.02" y="30.48"/>
<instance part="U$13" gate="G$1" x="33.02" y="50.8"/>
<instance part="U$14" gate="G$1" x="33.02" y="71.12"/>
<instance part="U$15" gate="G$1" x="33.02" y="91.44"/>
<instance part="U$16" gate="G$1" x="33.02" y="111.76"/>
<instance part="U$17" gate="G$1" x="33.02" y="132.08"/>
<instance part="U$18" gate="G$1" x="33.02" y="152.4"/>
<instance part="U$19" gate="G$1" x="33.02" y="172.72"/>
<instance part="U$20" gate="G$1" x="33.02" y="193.04"/>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<pinref part="U$10" gate="G$1" pin="GND"/>
<pinref part="U$20" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="198.12" x2="25.4" y2="198.12" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="U$10" gate="G$1" pin="PWR"/>
<pinref part="U$20" gate="G$1" pin="PWR"/>
<wire x1="17.78" y1="193.04" x2="25.4" y2="193.04" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="U$10" gate="G$1" pin="SIG"/>
<pinref part="U$20" gate="G$1" pin="SIG"/>
<wire x1="17.78" y1="187.96" x2="25.4" y2="187.96" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="U$9" gate="G$1" pin="GND"/>
<pinref part="U$19" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="177.8" x2="25.4" y2="177.8" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="U$9" gate="G$1" pin="PWR"/>
<pinref part="U$19" gate="G$1" pin="PWR"/>
<wire x1="17.78" y1="172.72" x2="25.4" y2="172.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="U$9" gate="G$1" pin="SIG"/>
<pinref part="U$19" gate="G$1" pin="SIG"/>
<wire x1="17.78" y1="167.64" x2="25.4" y2="167.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="U$8" gate="G$1" pin="GND"/>
<pinref part="U$18" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="157.48" x2="25.4" y2="157.48" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="U$8" gate="G$1" pin="PWR"/>
<pinref part="U$18" gate="G$1" pin="PWR"/>
<wire x1="17.78" y1="152.4" x2="25.4" y2="152.4" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="U$8" gate="G$1" pin="SIG"/>
<pinref part="U$18" gate="G$1" pin="SIG"/>
<wire x1="17.78" y1="147.32" x2="25.4" y2="147.32" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="U$7" gate="G$1" pin="GND"/>
<pinref part="U$17" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="137.16" x2="25.4" y2="137.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="U$7" gate="G$1" pin="PWR"/>
<pinref part="U$17" gate="G$1" pin="PWR"/>
<wire x1="17.78" y1="132.08" x2="25.4" y2="132.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="U$7" gate="G$1" pin="SIG"/>
<pinref part="U$17" gate="G$1" pin="SIG"/>
<wire x1="17.78" y1="127" x2="25.4" y2="127" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="U$6" gate="G$1" pin="GND"/>
<pinref part="U$16" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="116.84" x2="25.4" y2="116.84" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="U$6" gate="G$1" pin="PWR"/>
<pinref part="U$16" gate="G$1" pin="PWR"/>
<wire x1="17.78" y1="111.76" x2="25.4" y2="111.76" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="U$6" gate="G$1" pin="SIG"/>
<pinref part="U$16" gate="G$1" pin="SIG"/>
<wire x1="17.78" y1="106.68" x2="25.4" y2="106.68" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="U$5" gate="G$1" pin="GND"/>
<pinref part="U$15" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="96.52" x2="25.4" y2="96.52" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="U$5" gate="G$1" pin="PWR"/>
<pinref part="U$15" gate="G$1" pin="PWR"/>
<wire x1="17.78" y1="91.44" x2="25.4" y2="91.44" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<pinref part="U$5" gate="G$1" pin="SIG"/>
<pinref part="U$15" gate="G$1" pin="SIG"/>
<wire x1="17.78" y1="86.36" x2="25.4" y2="86.36" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="U$4" gate="G$1" pin="GND"/>
<pinref part="U$14" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="76.2" x2="25.4" y2="76.2" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="U$4" gate="G$1" pin="PWR"/>
<pinref part="U$14" gate="G$1" pin="PWR"/>
<wire x1="17.78" y1="71.12" x2="25.4" y2="71.12" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="U$4" gate="G$1" pin="SIG"/>
<pinref part="U$14" gate="G$1" pin="SIG"/>
<wire x1="17.78" y1="66.04" x2="25.4" y2="66.04" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$22" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="GND"/>
<pinref part="U$13" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="55.88" x2="25.4" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="PWR"/>
<pinref part="U$13" gate="G$1" pin="PWR"/>
<wire x1="17.78" y1="50.8" x2="25.4" y2="50.8" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="SIG"/>
<pinref part="U$13" gate="G$1" pin="SIG"/>
<wire x1="17.78" y1="45.72" x2="25.4" y2="45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="GND"/>
<pinref part="U$12" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="35.56" x2="25.4" y2="35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$26" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="PWR"/>
<pinref part="U$12" gate="G$1" pin="PWR"/>
<wire x1="17.78" y1="30.48" x2="25.4" y2="30.48" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$27" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="SIG"/>
<pinref part="U$12" gate="G$1" pin="SIG"/>
<wire x1="17.78" y1="25.4" x2="25.4" y2="25.4" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$28" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="GND"/>
<pinref part="U$11" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="15.24" x2="25.4" y2="15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$29" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="PWR"/>
<pinref part="U$11" gate="G$1" pin="PWR"/>
<wire x1="17.78" y1="10.16" x2="25.4" y2="10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$30" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="SIG"/>
<pinref part="U$11" gate="G$1" pin="SIG"/>
<wire x1="17.78" y1="5.08" x2="25.4" y2="5.08" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
