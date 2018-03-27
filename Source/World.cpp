/*
  ==============================================================================

  This is an automatically generated GUI class created by the Projucer!

  Be careful when adding custom code to these files, as only the code within
  the "//[xyz]" and "//[/xyz]" sections will be retained when the file is loaded
  and re-saved.

  Created with Projucer version: 5.2.0

  ------------------------------------------------------------------------------

  The Projucer is part of the JUCE library - "Jules' Utility Class Extensions"
  Copyright (c) 2015 - ROLI Ltd.

  ==============================================================================
*/

//[Headers] You can add your own extra header files here...
//[/Headers]

#include "World.h"


//[MiscUserDefs] You can add your own user definitions and misc code here...
//[/MiscUserDefs]

//==============================================================================
World::World ()
{
    //[Constructor_pre] You can add your own custom stuff here..
    //[/Constructor_pre]

    addAndMakeVisible (map = new Map());
    map->setName ("map");

    addAndMakeVisible (label17 = new Label ("new label",
                                            TRANS("Version 1.00")));
    label17->setFont (Font (15.00f, Font::plain).withTypefaceStyle ("Regular"));
    label17->setJustificationType (Justification::centredLeft);
    label17->setEditable (false, false, false);
    label17->setColour (TextEditor::textColourId, Colours::black);
    label17->setColour (TextEditor::backgroundColourId, Colour (0x00000000));

    addAndMakeVisible (textButton = new TextButton ("Run"));
    textButton->addListener (this);

    addAndMakeVisible (title = new TextEditor ("title"));
    title->setMultiLine (true);
    title->setReturnKeyStartsNewLine (true);
    title->setReadOnly (true);
    title->setScrollbarsShown (true);
    title->setCaretVisible (false);
    title->setPopupMenuEnabled (true);
    title->setText (TRANS("Extended Object Tracking Demo"));


    //[UserPreSize]
    //[/UserPreSize]

    setSize (1100, 600);


    //[Constructor] You can add your own custom stuff here..
    map->repaint();
    //[/Constructor]
}

World::~World()
{
    //[Destructor_pre]. You can add your own custom destruction code here..
    //[/Destructor_pre]

    map = nullptr;
    label17 = nullptr;
    textButton = nullptr;
    title = nullptr;


    //[Destructor]. You can add your own custom destruction code here..
    if (pSensor_ != nullptr) {
        pSensor_->removeChangeListener(this);
        delete pSensor_;
	}
	if (pUkf_ != nullptr) {
	    delete pUkf_;
	}

    //[/Destructor]
}

//==============================================================================
void World::paint (Graphics& g)
{
    //[UserPrePaint] Add your own custom painting code here..
    //[/UserPrePaint]

    g.fillAll (Colour (0xff505050));

    //[UserPaint] Add your own custom painting code here..
    //[/UserPaint]
}

void World::resized()
{
    //[UserPreResize] Add your own custom resize code here..
    //[/UserPreResize]

    map->setBounds (64, getHeight() - 64 - proportionOfHeight (0.8462f), proportionOfWidth (0.8469f), proportionOfHeight (0.8462f));
    label17->setBounds (1232, 8, 88, 24);
    textButton->setBounds (74, 31, 150, 24);
    title->setBounds (339, 32, 208, 24);
    //[UserResized] Add your own custom resize handling here..
    //[/UserResized]
}

void World::buttonClicked (Button* buttonThatWasClicked)
{
    //[UserbuttonClicked_Pre]
    //[/UserbuttonClicked_Pre]

    if (buttonThatWasClicked == textButton)
    {
        //[UserButtonCode_textButton] -- add your button handler code here..
        pSensor_ = new SensorUdp();
        pSensor_->reset();
        pSensor_->addChangeListener(this);
        pUkf_ = new UKF();
        //[/UserButtonCode_textButton]
    }

    //[UserbuttonClicked_Post]
    //[/UserbuttonClicked_Post]
}



//[MiscUserCode] You can add your own definitions of your custom methods or any other code here...


// ***************************************************************************
void World::plot(void) {
	map->repaint();
}


// ***************************************************************************
void World::changeListenerCallback(ChangeBroadcaster *)
{

    if (pSensor_ != nullptr)
    {
        // pass tm
        map->setTelemetry( pSensor_->getTelemetry() );
        pUkf_->ProcessMeasurement( pSensor_->getTelemetry() );
        map->setUkfInitialized( pUkf_->isInitialized() );
        map->setUkfState( pUkf_->getState() );
        map->setPose( pSensor_->getPose() );
        map->repaint();
        pSensor_->getNewData();
    }

}


//[/MiscUserCode]


//==============================================================================
#if 0
/*  -- Projucer information section --

    This is where the Projucer stores the metadata that describe this GUI layout, so
    make changes in here at your peril!

BEGIN_JUCER_METADATA

<JUCER_COMPONENT documentType="Component" className="World" componentName="" parentClasses="public Component, public ChangeListener"
                 constructorParams="" variableInitialisers="" snapPixels="8" snapActive="1"
                 snapShown="1" overlayOpacity="0.330" fixedSize="0" initialWidth="1100"
                 initialHeight="600">
  <BACKGROUND backgroundColour="ff505050"/>
  <GENERICCOMPONENT name="map" id="dc308528de253f76" memberName="map" virtualName=""
                    explicitFocusOrder="0" pos="64 64Rr 84.694% 84.615%" class="Map"
                    params=""/>
  <LABEL name="new label" id="83dd3fe07f3590c0" memberName="label17" virtualName=""
         explicitFocusOrder="0" pos="1232 8 88 24" edTextCol="ff000000"
         edBkgCol="0" labelText="Version 1.00" editableSingleClick="0"
         editableDoubleClick="0" focusDiscardsChanges="0" fontname="Default font"
         fontsize="15" kerning="0" bold="0" italic="0" justification="33"/>
  <TEXTBUTTON name="Run" id="38bc2ca22e9982c" memberName="textButton" virtualName=""
              explicitFocusOrder="0" pos="74 31 150 24" buttonText="Run" connectedEdges="0"
              needsCallback="1" radioGroupId="0"/>
  <TEXTEDITOR name="title" id="44247fd3237e81bb" memberName="title" virtualName=""
              explicitFocusOrder="0" pos="339 32 208 24" initialText="Extended Object Tracking Demo"
              multiline="1" retKeyStartsLine="1" readonly="1" scrollbars="1"
              caret="0" popupmenu="1"/>
</JUCER_COMPONENT>

END_JUCER_METADATA
*/
#endif


//[EndFile] You can add extra defines here...
//[/EndFile]
