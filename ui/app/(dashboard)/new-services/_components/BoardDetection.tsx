/**
 * @file BoardDetection.tsx
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief Board Detection service component
 *
 * @copyright Copyright (c) 2024
 *
 */
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
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { ScrollArea } from "@/components/ui/scroll-area";
import { board_detection_service } from "@/utils/services";
import { Button } from "@/components/ui/button";
import { Phone } from "lucide-react";

const BoardDetection = () => {
  return (
    <>
      <Dialog>
        <DialogTrigger asChild>
          <Card className="cursor-pointer hover:bg-accent transition-colors h-fit bg-white">
            <CardHeader>
              <CardTitle>Board Detection</CardTitle>
              <CardDescription className="line-clamp-2">
                Used to get the board transformation matrix
              </CardDescription>
            </CardHeader>
          </Card>
        </DialogTrigger>
        <DialogContent className="sm:max-w-[425px] bg-white">
          <DialogHeader>
            <DialogTitle>/board_detection</DialogTitle>
          </DialogHeader>
          <div className="space-y-4">
            <ScrollArea className="h-[200px] w-full rounded-md border p-4">
              <h3 className="font-semibold mb-2">Description</h3>
              <p className="text-sm text-muted-foreground">
                {" "}
                Board detection service is used to get the board transformation
                matrix
              </p>
              <h3 className="font-semibold mt-4 mb-2">Usage</h3>
              <p className="text-sm text-muted-foreground">
                This service can be called without any arguments. It performs an
                immediate action and returns a boolean result.
              </p>
              <h3 className="font-semibold mt-4 mb-2">Return Value</h3>
              <p className="text-sm text-muted-foreground">
                The service returns true if the operation was successful, and
                false otherwise.
              </p>
            </ScrollArea>
            <Button
              onClick={board_detection_service}
              className="w-full bg-[#f0721b] text-white"
            >
              <Phone className="mr-2 h-4 w-4" /> Call Service
            </Button>
          </div>
        </DialogContent>
      </Dialog>
    </>
  );
};

export default BoardDetection;
