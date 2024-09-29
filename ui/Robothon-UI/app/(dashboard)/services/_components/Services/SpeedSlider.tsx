"use client";
import React, {useState} from 'react'
import { cn } from "@/lib/utils"
import { Slider } from "@/components/ui/slider"
import { set_speed_slider_service } from "@/utils/services"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "@/components/ui/popover"


type SliderProps = React.ComponentProps<typeof Slider>


const SpeedSlider = () => {

    const [sliderValue, setSliderValue] = useState(0); // Initial slider value

    const handleSliderChange = (value: number[]) => {
        const newValue = value[0];
        setSliderValue(newValue); // Update local state
        set_speed_slider_service({ slider_args: newValue }); // Call the service with new value
    };
    
    return (
        <Popover>
            <PopoverTrigger asChild>
            <Button variant="outline">Set Speed Slider</Button>
            </PopoverTrigger>
            <PopoverContent className="w-80">
            <div className='flex gap-3 items-center'><Label className=''>Speed Slider :</Label> <span>{sliderValue}</span></div>
            <Slider
                min={0}
                max={1}
                step={0.1}
                defaultValue={[sliderValue]} // Set initial value
                onValueChange={handleSliderChange} // Handle value change
                className='mt-2'
            />
            </PopoverContent>
        </Popover>
    )
}

export default SpeedSlider