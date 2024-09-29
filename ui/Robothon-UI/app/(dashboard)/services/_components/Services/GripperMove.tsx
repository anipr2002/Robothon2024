"use client";
import React, { useState } from 'react';
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "@/components/ui/popover";
import { set_gripper_service } from '@/utils/services';
import useDebounce from '@/hooks/use-debounce'; 
import { useServiceStore } from '@/store/serviceStore';

export function GripperMove() {


    const { gripper, setGripper } = useServiceStore();
    // Create debounced values for each gripper parameter
    const debouncedGripper = useDebounce(gripper, 300); // Adjust the delay as needed

    const handleGripperChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const { name, value } = e.target;

        // Parse the input value and default to 0.0 if NaN
        const newValue = parseFloat(value);
        const validValue = isNaN(newValue) ? 0.0 : newValue;
        type GripperArgProps = { position_unit: number, position: number, speed: number, force: number, asynchronous: boolean };

        setGripper({ ...gripper, [name]: validValue } as GripperArgProps);

    };

    // Call the service whenever the debounced value changes
    React.useEffect(() => {
        set_gripper_service(debouncedGripper);
    }, [debouncedGripper]);

    return (
        <Popover>
            <PopoverTrigger asChild>
                <Button variant="outline">Gripper</Button>
            </PopoverTrigger>
            <PopoverContent className="w-80">
                <div className="grid gap-4">
                    <div className="space-y-2">
                        <h4 className="font-medium leading-none">Gripper Settings</h4>
                        <p className="text-sm text-muted-foreground">
                            Set the gripper parameters.
                        </p>
                    </div>
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
                </div>
            </PopoverContent>
        </Popover>
    );
}
