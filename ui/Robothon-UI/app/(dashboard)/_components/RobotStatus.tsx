"use client";
import React from "react";
import { Card, CardHeader, CardTitle, CardContent } from "@/components/ui/card";
import { useRobotStore } from "@/store/robotStore";

const RobotStatus = () => {
  const { robotPose } = useRobotStore();
  return (
    <div className="absolute bottom-4 right-5">
      <Card className="cursor-pointer hover:bg-accent transition-colors h-fit bg-white">
        <CardHeader>
          <CardTitle className="text-4xl font-roger">Robot Position</CardTitle>
        </CardHeader>
        <CardContent className="p-4">
          <div className="flex gap-3 items-center">
            <div className="text-xl font-roger">
              X: {robotPose.translation.x.toFixed(2)}
            </div>
            <div className="text-xl font-roger">
              Y: {robotPose.translation.y.toFixed(2)}
            </div>
            <div className="text-xl font-roger">
              Z: {robotPose.translation.z.toFixed(2)}
            </div>
          </div>
          <div className="flex gap-3 items-center">
            <div className="text-xl font-roger">Rx: {robotPose.rotation.x.toFixed(2)}</div>
            <div className="text-xl font-roger">Ry: {robotPose.rotation.y.toFixed(2)}</div>
            <div className="text-xl font-roger">Rz: {robotPose.rotation.z.toFixed(2)}</div>
            <div className="text-xl font-roger">Rw: {robotPose.rotation.w.toFixed(2)}</div>
          </div>
        </CardContent>
      </Card>
    </div>
  );
};

export default RobotStatus;
