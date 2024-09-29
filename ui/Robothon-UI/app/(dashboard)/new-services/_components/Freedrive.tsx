
"use client";
import React, { useState } from 'react';
import { Dialog, DialogContent, DialogDescription, DialogHeader, DialogTitle, DialogTrigger } from "@/components/ui/dialog";
import { Card, CardHeader, CardTitle, CardDescription } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Checkbox } from "@/components/ui/checkbox";
import { Label } from "@/components/ui/label";
import { Button } from "@/components/ui/button";
import { set_freedive_service } from "@/utils/services";

const Freedrive = () => {
  const [freeAxes, setFreeAxes] = useState([false, false, false, false, false, false]);

  const handleCheckboxChange = (index: number) => {
    setFreeAxes((prev) => {
      const newAxes = [...prev];
      newAxes[index] = !newAxes[index];
      return newAxes;
    });
  };

  const handleSubmit = () => {
    const args = {
      IO: true,
      free_axes: freeAxes.map(axis => axis ? 1 : 0),
      feature: 1,
      custom_frame: {
        translation: { x: 0, y: 0, z: 0 },
        rotation: { x: 0, y: 0, z: 0, w: 1 }
      }
    };
    console.log("Submitting freedrive args:", args);
    // Here you would call your set_freedrive_service function
    set_freedive_service({ args });
  };

  return (
    <Dialog>
      <DialogTrigger asChild>
        <Card className="cursor-pointer hover:bg-accent transition-colors h-fit">
          <CardHeader>
            <CardTitle>Freedrive</CardTitle>
            <CardDescription className="line-clamp-2">
              Change the constraints of the robot to allow for manual control
            </CardDescription>
          </CardHeader>
        </Card>
      </DialogTrigger>
      <DialogContent className="sm:max-w-[425px] bg-white">
        <DialogHeader>
          <DialogTitle>Set Freedrive</DialogTitle>
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
                This service allows you to change the constraints of the robot, enabling manual control of specific axes.
              </p>
              <h3 className="font-semibold mt-4 mb-2">Usage</h3>
              <p className="text-sm text-muted-foreground">
                Select the axes you want to free for manual control. When enabled, you can physically move the robot along these axes.
              </p>
              <h3 className="font-semibold mt-4 mb-2">Return Value</h3>
              <p className="text-sm text-muted-foreground">
                This service returns a void value. The effect is immediately applied to the robot's movement constraints.
              </p>
            </ScrollArea>
          </TabsContent>
          <TabsContent value="args">
            <ScrollArea className="h-[200px] w-full rounded-md border p-4">
              <div className="space-y-4">
                <h3 className="font-semibold mb-2">Free Axes</h3>
                <p className="text-sm text-muted-foreground mb-4">
                  Select which axes to free for manual control:
                </p>
                {['X', 'Y', 'Z', 'RX', 'RY', 'RZ'].map((axis, index) => (
                  <div key={axis} className="flex items-center space-x-2">
                    <Checkbox 
                      id={`axis-${axis}`} 
                      checked={freeAxes[index]}
                      onCheckedChange={() => handleCheckboxChange(index)}
                    />
                    <Label htmlFor={`axis-${axis}`}>{axis} Axis</Label>
                  </div>
                ))}
              </div>
            </ScrollArea>
            <div className="mt-4 flex justify-end">
              <Button onClick={handleSubmit}>Apply Freedrive</Button>
            </div>
          </TabsContent>
        </Tabs>
      </DialogContent>
    </Dialog>
  );
};

export default Freedrive;