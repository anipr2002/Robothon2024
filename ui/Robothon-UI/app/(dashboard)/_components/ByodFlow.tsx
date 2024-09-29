// @ts-nocheck

/**
 * @file ByodFlow.tsx
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief BYOD Flow component
 *
 * @copyright Copyright (c) 2024
 *
 */

"use client";
import React, { useCallback, useState } from "react";
import {
  ReactFlow,
  addEdge,
  MiniMap,
  Controls,
  Background,
  useNodesState,
  useEdgesState,
  MarkerType,
  Handle,
  Position,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";

import { useStore } from "@/store/store";
import { useToast } from "@/hooks/use-toast";
import { set_speed_slider_service } from "@/utils/services";

import { Card } from "@/components/ui/card";
import { AnimatedSubscribeButton } from "@/components/animatedbutton";
import { CheckIcon, ChevronRightIcon } from "lucide-react";

const TaskNode = ({ data }: { data: { label: string } }) => (
  <Card className="px-4 py-2 shadow-md bg-white">
    <div className="font-bold text-sm">{data.label}</div>
    <Handle
      type="target"
      position={Position.Left}
      style={{
        backgroundColor: "#f0721b",
      }}
      //   style={{ left: "-8px", top: "50%" }}
    />
    <Handle
      type="source"
      position={Position.Right}
      style={{
        backgroundColor: "#f0721b",
      }}
      //   style={{ right: "-8px", top: "50%" }}
    />
  </Card>
);

const StartNode = ({ data }: { data: { label: string } }) => (
  <Card className="px-4 py-2 bg-[#f0721b]">
    <div className="font-bold text-sm">{data.label}</div>
    <Handle
      type="target"
      position={Position.Left}
      style={{
        backgroundColor: "#f0721b",
      }}
      //   style={{ left: "-8px", top: "50%" }}
    />
    <Handle
      type="source"
      position={Position.Right}
      style={{
        backgroundColor: "#f0721b",
      }}
      //   style={{ right: "-8px", top: "50%" }}
    />
  </Card>
);

const nodeTypes = {
  taskNode: TaskNode,
  startNode: StartNode,
};

const initialNodes = [
  {
    id: "start",
    position: { x: 0, y: 75 },
    data: { label: "Start" },
    style: { background: "#6ede87", color: "white" },
    // sourcePosition: "right",
    // targetPosition: "left",
    type: "startNode",
  },
  {
    id: "1",
    position: { x: 200, y: 0 },
    data: { label: "Charging Status Slot 2" },
    type: "taskNode",
  },
  {
    id: "2",
    position: { x: 400, y: 0 },
    data: { label: "Remove Phone" },
    type: "taskNode",
  },
  {
    id: "3",
    position: { x: 600, y: 0 },
    data: { label: "Blow Hot Air" },
    type: "taskNode",
  },
  {
    id: "4",
    position: { x: 200, y: 150 },
    data: { label: "Remove Back Cover" },
    type: "taskNode",
  },
  {
    id: "5",
    position: { x: 400, y: 150 },
    data: { label: "Sort Phone" },
    type: "taskNode",
  },
  {
    id: "6",
    position: { x: 600, y: 150 },
    data: { label: "Task 6" },
    type: "taskNode",
  },
  {
    id: "7",
    position: { x: 200, y: 0 },
    data: { label: "Charging Status Slot 1" },
    type: "taskNode",
  },
];

const ByodFlow = () => {
  const [nodes, setNodes, onNodesChange] = useNodesState(initialNodes);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);
  const [isPlaying, setIsPlaying] = useState(true);
  const { tasks, setTasks } = useStore();
  const { toast } = useToast();

  //   const [taskOrder, setTaskOrder] = useState([]);

  const togglePlayPause = () => {
    const newSpeed = isPlaying ? 0 : 0.2;
    setIsPlaying(!isPlaying);

    set_speed_slider_service({ slider_args: newSpeed });
  };

  const onConnect = useCallback(
    (params) => {
      // Check if the source is the start node and it already has an outgoing edge
      if (
        params.source === "start" &&
        edges.some((edge) => edge.source === "start")
      ) {
        return; // Prevent multiple connections from start
      }

      // Check if the target node already has an input
      const targetHasInput = edges.some(
        (edge) => edge.target === params.target
      );
      if (!targetHasInput && params.target !== "start") {
        // Prevent connections to start node
        const newEdge = {
          ...params,
          type: "smoothstep",
          animated: true,
          label: `${edges.length + 1}`,
          labelStyle: { fill: "#000", fontWeight: 700, fontSize: 16 },
          labelBgStyle: { fill: "rgba(255, 255, 255, 0.75)" },
          markerEnd: {
            type: MarkerType.ArrowClosed,
            width: 20,
            height: 20,
            color: "#f0721b",
          },
          style: {
            stroke: "#f0721b",
          },
        };
        setEdges((eds) => addEdge(newEdge, eds));
      }
    },
    [edges]
  );

  const getTaskOrder = (): number[] | null => {
    const order = [];
    const visited = new Set();
    const tempVisited = new Set();

    const dfs = (nodeId) => {
      if (tempVisited.has(nodeId)) {
        // Cycle detected
        return false;
      }
      if (visited.has(nodeId)) {
        return true;
      }
      visited.add(nodeId);
      tempVisited.add(nodeId);

      const outgoingEdges = edges.filter((edge) => edge.source === nodeId);
      for (const edge of outgoingEdges) {
        if (!dfs(edge.target)) {
          return false;
        }
      }

      tempVisited.delete(nodeId);
      order.unshift(nodeId);
      return true;
    };

    // Start with the 'start' node
    if (!dfs("start")) {
      return null; // Cycle detected or start node not connected
    }

    // Process remaining nodes
    for (const node of nodes) {
      if (!visited.has(node.id)) {
        if (!dfs(node.id)) {
          return null;
        }
      }
    }

    return order;
  };

  const handleGetOrder = async () => {
    const order = getTaskOrder();
    if (order) {
      const taskOrder = order.map((id) => (id === "start" ? 0 : Number(id)));
      setTasks(taskOrder);

      console.log(JSON.stringify({ tasks: taskOrder }));
      try {
        const response = await fetch("http://localhost:8080/post", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ tasks: [11, 12, 13] }),
        });

        if (response.ok) {
          toast({
            title: "Success 🎉",
            description: "Task order sent successfully",
            duration: 3000,
          });
        } else {
          throw new Error("Failed to send task order");
        }
      } catch (error) {
        console.error("Error:", error);
        toast({
          title: "Error",
          description: "Failed to send task order",
          variant: "destructive",
          duration: 3000,
        });
      }
    } else {
      setTasks([]);
      toast({
        title: "Invalid Flow",
        description:
          "Please check your connections and ensure the Start node is connected.",
        variant: "destructive",
        duration: 3000,
      });
    }
  };

  return (
    <div style={{ height: "100%", width: "100%" }}>
      <ReactFlow
        nodes={nodes}
        edges={edges}
        onNodesChange={onNodesChange}
        onEdgesChange={onEdgesChange}
        onConnect={onConnect}
        nodeTypes={nodeTypes}
        fitView
      >
        <Controls />
        <MiniMap />
        <Background variant="cross" gap={12} size={1} />
      </ReactFlow>
      <div className="absolute bottom-4 left-1/2 transform -translate-x-1/2">
        {/* <Button className="shadow-lg" onClick={handleGetOrder}>
          <PlayIcon className="mr-2 h-4 w-4" /> Perform Tasks
        </Button> */}
        <AnimatedSubscribeButton
          buttonColor="#000000"
          buttonTextColor="#ffffff"
          subscribeStatus={false}
          onClick={handleGetOrder}
          initialText={
            <span className="group inline-flex items-center">
              Perform Tasks{" "}
              <ChevronRightIcon className="ml-1 h-4 w-4 transition-transform duration-300 group-hover:translate-x-1" />
            </span>
          }
          changeText={
            <span className="group inline-flex items-center">
              <CheckIcon className="mr-2 h-4 w-4" />
              Done{" "}
            </span>
          }
        />
      </div>
    </div>
  );
};

export default ByodFlow;
