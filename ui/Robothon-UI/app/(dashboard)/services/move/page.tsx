"use client"
import React from 'react'
import tf2 from '@/utils/tf2.json'
import { Button } from '@/components/ui/button';
import { set_freedive_args, set_freedive_service } from '@/utils/services';

interface TaskBoardItem {
  parent: string;
  name: string;
  transform: any
}

const TaskBoardButtons = ({ data } : {data: Record<string, TaskBoardItem>}) => {
  // Filter the buttons based on the "parent" being "task_board"
  const taskBoardButtons = Object.values(data).filter((item: TaskBoardItem) => (item.parent === 'task_board') || (item.parent === 'base'));

  const IO = false;

  const handleClick = (transform: number[][], parent:string) => {
    // Extract translation and rotation from the transform matrix
    const translation = {
      x: transform[0][3], // X from the matrix
      y: transform[1][3], // Y from the matrix
      z: transform[2][3]  // Z from the matrix
    };

    // Extract rotation from the matrix (for now, assuming identity quaternion)
    // You may want to convert the rotation matrix into quaternion for real-world usage.
    const rotation = {
      x: 0, // Placeholder, should be calculated from the matrix if needed
      y: 0,
      z: 0,
      w: 1 // Identity quaternion
    };

    // Prepare the service args
    const args: set_freedive_args = {
      IO: true, // Example value, modify as per your use case
      free_axes: [1, 1, 1, 1, 1, 1], // Example value, modify as per your use case
        //if item.parent === 'base feature = 0 else feature = 2
      feature: parent === 'base' ? 0 : 2,
        custom_frame: {
        translation,
        rotation
      }
    };

    // Call the ROS service
    set_freedive_service({ args });
  };


  return (
    <div>
      {taskBoardButtons.map((item, index) => (
        <Button key={index} variant="default" className="m-2" onClick={()=>handleClick(item.transform, item.parent)}>
          {item.name}
        </Button>
      ))}
    </div>
  );
};


const page = () => {
  return (
    <div className='w-full h-screen flex flex-col p-8'>
      <div className='text-3xl font-bold'>Positions</div>

      <div className='mt-5 flex gap-4'>
        <TaskBoardButtons data={tf2} />
      </div>
    </div>
  )
}

export default page