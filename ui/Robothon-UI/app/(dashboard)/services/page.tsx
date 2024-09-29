//@ts-nocheck
"use client";
import React, { useState } from "react";
import * as ROSLIB from "roslib";
import ros from "@/utils/ros";
import { Button } from "@/components/ui/button";
import SpeedSlider from "./_components/Services/SpeedSlider";
import BoardDetection from "./_components/Services/BoardDetection";
import { GripperMove } from "./_components/Services/GripperMove";
import { redirect } from "next/navigation";
import Link from "next/link";

const page = () => {
  return (
    <div className="w-full h-screen flex flex-col p-8">
      <div className="text-3xl font-bold">SERVICES</div>

      <div className="mt-5 flex gap-4">
        <SpeedSlider />
        <BoardDetection />
        <GripperMove />
        {/* <Button onClick={redirect('/services/move')}>Move</Button> */}
        <Link href={"/services/move"}>
          <Button>Move</Button>
        </Link>
      </div>
    </div>
  );
};

export default page;
