/**
 * @file page.tsx
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief Main page component
 *
 * @copyright Copyright (c) 2024
 *
 */
import React from "react";

const Page = () => {
  return (
    <>
      <div className="w-full h-screen flex items-center justify-center">
        <div className="flex flex-col items-center justify-center gap-10">
          <h1 className="text-8xl font-roger font-bold">
            Welcome to Robothon 2024
          </h1>
          <h1 className="text-8xl  font-roger font-bold">
            Team <span className="text-[#f0721b]">RoboPig</span>
          </h1>
        </div>
      </div>
      <div
        aria-hidden="true"
        className=" pointer-events-none absolute inset-x-0 -top-40 -z-10 transform-gpu overflow-hidden blur-3xl sm:-top-80"
      >
        <div
          style={{
            clipPath:
              "polygon(74.1% 44.1%, 100% 61.6%, 97.5% 26.9%, 85.5% 0.1%, 80.7% 2%, 72.5% 32.5%, 60.2% 62.4%, 52.4% 68.1%, 47.5% 58.3%, 45.2% 34.5%, 27.5% 76.7%, 0.1% 64.9%, 17.9% 100%, 27.6% 76.8%, 76.1% 97.7%, 74.1% 44.1%)",
          }}
          className="relative left-[calc(50%-11rem)] aspect-[1155/678] w-[36.125rem] -translate-x-1/2 rotate-[30deg] bg-gradient-to-r from-red-500 to-orange-500 opacity-30 sm:left-[calc(50%-30rem)] sm:w-[72.1875rem]"
        />
      </div>
    </>
  );
};

export default Page;
