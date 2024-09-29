"use client";
import React from "react";
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

import { set_gripper_service } from "@/utils/services";
import useDebounce from "@/hooks/use-debounce";
import { useServiceStore } from "@/store/serviceStore";
import { Input } from "@/components/ui/input";

const GripperMove = () => {
  const { gripper, setGripper } = useServiceStore();
  // Create debounced values for each gripper parameter
  const debouncedGripper = useDebounce(gripper, 300); // Adjust the delay as needed

  const handleGripperChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;

    // Parse the input value and default to 0.0 if NaN
    const newValue = parseFloat(value);
    const validValue = isNaN(newValue) ? 0.0 : newValue;
    type GripperArgProps = {
      position_unit: number;
      position: number;
      speed: number;
      force: number;
      asynchronous: boolean;
    };

    setGripper({ ...gripper, [name]: validValue } as GripperArgProps);
  };

  // Call the service whenever the debounced value changes
  React.useEffect(() => {
    set_gripper_service(debouncedGripper);
  }, [debouncedGripper]);

  return (
    <>
      <Dialog>
        <DialogTrigger asChild>
          <Card className="cursor-pointer hover:bg-accent transition-colors h-fit">
            <CardHeader>
              <CardTitle>Gripper Move</CardTitle>
              <CardDescription className="line-clamp-2">
                Change the gripper parameters
              </CardDescription>
            </CardHeader>
          </Card>
        </DialogTrigger>
        <DialogContent className="sm:max-w-[425px] bg-white">
          <DialogHeader>
            <DialogTitle>set_gripper</DialogTitle>
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
                  This service allows you to change the gripper parameters.
                </p>
                {/* Add more detailed information here */}
                <h3 className="font-semibold mt-4 mb-2">Usage</h3>
                <p className="text-sm text-muted-foreground">
                  This service can be called using the ROS command line tools or
                  through a ROS node.
                </p>
                <h3 className="font-semibold mt-4 mb-2">Return Value</h3>
                <p className="text-sm text-muted-foreground">
                  This service returns a void value.
                </p>
              </ScrollArea>
            </TabsContent>
            <TabsContent value="args">
              <ScrollArea className="h-[200px] w-full rounded-md border p-4">
                <div className="grid gap-2">
                  <div className="grid grid-cols-3 items-center gap-4">
                    <Label htmlFor="position_unit">Position Unit</Label>
                    <Input
                      id="position_unit"
                      name="position_unit"
                      value={gripper.position_unit.toString()}
                      onChange={handleGripperChange}
                      className="col-span-2 h-8"
                    />
                  </div>
                  <div className="grid grid-cols-3 items-center gap-4">
                    <Label htmlFor="position">Position</Label>
                    <Input
                      id="position"
                      name="position"
                      value={gripper.position.toString()}
                      onChange={handleGripperChange}
                      className="col-span-2 h-8"
                    />
                  </div>
                  <div className="grid grid-cols-3 items-center gap-4">
                    <Label htmlFor="speed">Speed</Label>
                    <Input
                      id="speed"
                      name="speed"
                      value={gripper.speed.toString()}
                      onChange={handleGripperChange}
                      className="col-span-2 h-8"
                    />
                  </div>
                  <div className="grid grid-cols-3 items-center gap-4">
                    <Label htmlFor="force">Force</Label>
                    <Input
                      id="force"
                      name="force"
                      value={gripper.force.toString()}
                      onChange={handleGripperChange}
                      className="col-span-2 h-8"
                    />
                  </div>
                </div>
              </ScrollArea>
            </TabsContent>
          </Tabs>
        </DialogContent>
      </Dialog>
    </>
  );
};

export default GripperMove;
