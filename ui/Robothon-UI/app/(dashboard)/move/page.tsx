/**
 * @file page.tsx
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief Move page component
 *
 * @copyright Copyright (c) 2024
 *
 */
"use client";

import React from "react";
import tf2 from "@/utils/tf2.json";
import { Button } from "@/components/ui/button";
import {
  set_cart_target_args,
  set_cart_target_service,
} from "@/utils/services";

interface TaskBoardItem {
  parent: string;
  name: string;
  transform: number[][];
}

const TaskBoardButtons = ({
  data,
}: {
  data: Record<string, TaskBoardItem>;
}) => {
  // Filter the buttons based on the "parent" being "byod_board" or "base"
  const taskBoardButtons = Object.values(data).filter(
    (item: TaskBoardItem) =>
      item.parent === "byod_board" || item.parent === "base"
  );

  function matrixToQuaternionAndTranslation(matrix: number[][]): {
    x: number;
    y: number;
    z: number;
    w: number;
    tx: number;
    ty: number;
    tz: number;
  } {
    // Extract the 3x3 rotation matrix from the 4x4 matrix
    const m00 = matrix[0][0],
      m01 = matrix[0][1],
      m02 = matrix[0][2];
    const m10 = matrix[1][0],
      m11 = matrix[1][1],
      m12 = matrix[1][2];
    const m20 = matrix[2][0],
      m21 = matrix[2][1],
      m22 = matrix[2][2];

    // Compute quaternion
    const w = Math.sqrt(1.0 + m00 + m11 + m22) / 2.0;
    const x = (m21 - m12) / (4.0 * w);
    const y = (m02 - m20) / (4.0 * w);
    const z = (m10 - m01) / (4.0 * w);

    // Extract translation from the last column of the 4x4 matrix
    const tx = matrix[0][3];
    const ty = matrix[1][3];
    const tz = matrix[2][3];

    return { x, y, z, w, tx, ty, tz };
  }

  const handleClick = (transform: number[][], parent: string) => {
    // Extract translation and rotation from the transform matrix
    const { x, y, z, w, tx, ty, tz } =
      matrixToQuaternionAndTranslation(transform);

    // Prepare the service args
    const args: set_cart_target_args = {
      mode: 1, // Assuming mode 1 is for cartesian movement
      speed: 0.25,
      acceleration: 0.25,
      asynchronous: false,
      cartesian_goal: {
        translation: { x: tx, y: ty, z: tz },
        rotation: { x, y, z, w },
      },
    };

    console.log("args", args);
    // Call the ROS service
    set_cart_target_service({ args });
  };

  return (
    <div>
      {taskBoardButtons.map((item, index) => (
        <Button
          key={index}
          variant="default"
          className="m-2"
          onClick={() => handleClick(item.transform, item.parent)}
        >
          {item.name}
        </Button>
      ))}
    </div>
  );
};

const Page = () => {
  return (
    <div className="w-full h-screen flex flex-col p-8">
      <div className="text-3xl font-bold">Positions</div>
      <div className="mt-5 flex gap-4">
        <TaskBoardButtons data={tf2} />
      </div>
    </div>
  );
};

export default Page;
