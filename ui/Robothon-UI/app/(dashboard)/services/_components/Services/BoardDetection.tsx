import React, { useState } from 'react';
import { Button } from '@/components/ui/button'; // Adjust the import based on your project structure
import { AlertDialog, AlertDialogTrigger, AlertDialogContent, AlertDialogTitle, AlertDialogDescription, AlertDialogAction, AlertDialogCancel } from '@/components/ui/alert-dialog'; // Adjust based on your UI library
import { board_detection_service } from '@/utils/services';

const BoardDetection = () => {
    const [isOpen, setIsOpen] = useState(false); // State to manage dialog visibility

    const handleBoardDetection = () => {
        // Call the board detection service
        board_detection_service();
        setIsOpen(false); // Close dialog after calling service
    };

    return (
        <div>
            <AlertDialog open={isOpen} onOpenChange={setIsOpen}>
                <AlertDialogTrigger asChild>
                    <Button variant="outline">Start Board Detection</Button>
                </AlertDialogTrigger>
                <AlertDialogContent>
                    <AlertDialogTitle>Confirm Board Detection</AlertDialogTitle>
                    <AlertDialogDescription>
                        Are you sure you want to start the board detection process?
                    </AlertDialogDescription>
                    <div className="flex justify-end space-x-2">
                        <AlertDialogCancel onClick={() => setIsOpen(false)}>Cancel</AlertDialogCancel>
                        <AlertDialogAction onClick={handleBoardDetection}>Call 📞</AlertDialogAction>
                    </div>
                </AlertDialogContent>
            </AlertDialog>
        </div>
    );
};

export default BoardDetection;
