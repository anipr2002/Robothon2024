/**
 * @file SpeedSlider.tsx
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief Speed Slider service component
 *
 * @copyright Copyright (c) 2024
 *
 */
"use client";
import React, { useState } from "react";
import { Slider } from "@/components/ui/slider";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";

import {
  Card,
  CardContent,
  CardDescription,
  CardFooter,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Label } from "@/components/ui/label";
import { set_speed_slider_service } from "@/utils/services";

const SpeedSlider = () => {
  const [sliderValue, setSliderValue] = useState(0); // Initial slider value

  const handleSliderChange = (value: number[]) => {
    const newValue = value[0];
    setSliderValue(newValue); // Update local state
    set_speed_slider_service({ slider_args: newValue }); // Call the service with new value
  };
  return (
    <>
      <Dialog>
        <DialogTrigger asChild>
          <Card className="cursor-pointer hover:bg-accent transition-colors h-fit">
            <CardHeader>
              <CardTitle>Speed Slider</CardTitle>
              <CardDescription className="line-clamp-2">
                Change the speed of the robot
              </CardDescription>
            </CardHeader>
          </Card>
        </DialogTrigger>
        <DialogContent className="sm:max-w-[425px] bg-white">
          <DialogHeader>
            <DialogTitle>set_speed_slider</DialogTitle>
          </DialogHeader>
          <Tabs defaultValue="info" className="w-full">
            <TabsList className="grid w-full grid-cols-2">
              <TabsTrigger value="info">Information</TabsTrigger>
              <TabsTrigger value="args">Arguments</TabsTrigger>
            </TabsList>
            <TabsContent value="info">
              <ScrollArea className="h-[200px] w-full rounded-md border p-4">
                <h3 className="font-semibold mb-2">Description</h3>
                <p className="text-sm text-muted-foreground">
                  This service allows you to change the speed of the robot. The
                  speed is a value between 0 and 1, where 0 is stopped and 1 is
                  the fastest speed.
                </p>
                {/* Add more detailed information here */}
                <h3 className="font-semibold mt-4 mb-2">Usage</h3>
                <p className="text-sm text-muted-foreground">
                  This service can be called using the ROS command line tools or
                  through a ROS node.
                </p>
                <h3 className="font-semibold mt-4 mb-2">Return Value</h3>
                <p className="text-sm text-muted-foreground">
                  The service returns a boolean indicating success or failure of
                  the operation.
                </p>
              </ScrollArea>
            </TabsContent>
            <TabsContent value="args">
              <ScrollArea className="h-[200px] w-full rounded-md border p-4">
                <Slider
                  min={0}
                  max={1}
                  step={0.1}
                  defaultValue={[sliderValue]} // Set initial value
                  onValueChange={handleSliderChange} // Handle value change
                  className="mt-2"
                />
                <div className="flex gap-3 items-center mt-6">
                  <Label className="text-2xl font-medium">Speed Slider :</Label>{" "}
                  <span className="text-2xl">{sliderValue}</span>
                </div>
              </ScrollArea>
            </TabsContent>
          </Tabs>
        </DialogContent>
      </Dialog>
    </>
  );
};

export default SpeedSlider;
