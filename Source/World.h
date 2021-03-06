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

#pragma once

//[Headers]     -- You can add your own extra header files here --
#include "JuceHeader.h"
#include "Map.h"
#include "SensorUdp.h"
#include "Kf.h"
//[/Headers]



//==============================================================================
/**
                                                                    //[Comments]
    An auto-generated component, created by the Introjucer.

    Describe your class and how it works here!
                                                                    //[/Comments]
*/
class World  : public Component,
               public ChangeListener,
               public Button::Listener
{
public:
    //==============================================================================
    World ();
    ~World();

    //==============================================================================
    //[UserMethods]     -- You can add your own custom methods in this section.

	void plot(void);
	void changeListenerCallback(ChangeBroadcaster *) override;

    //[/UserMethods]

    void paint (Graphics& g) override;
    void resized() override;
    void buttonClicked (Button* buttonThatWasClicked) override;



private:
    //[UserVariables]   -- You can add your own custom variables in this section.

    SensorUdp   *pSensor_;
    KF         *pKf_;
    SensorUdpTelemetry tm_;

    //[/UserVariables]

    //==============================================================================
    ScopedPointer<Map> map;
    ScopedPointer<Label> label17;
    ScopedPointer<TextButton> textButton;
    ScopedPointer<TextEditor> title;


    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (World)
};

//[EndFile] You can add extra defines here...
//[/EndFile]
