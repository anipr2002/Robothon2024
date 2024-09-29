import { NextRequest, NextResponse } from 'next/server';

export async function POST(request: NextRequest) {
  try {
    // Parse the JSON body from the request
    const body = await request.json();

    // Log the received tasks to the console
    console.log('Received tasks:', body.tasks);

    // You can process the tasks here if needed

    // Return a success response
    return NextResponse.json({ message: 'Tasks received successfully' }, { status: 200 });
  } catch (error) {
    console.error('Error processing request:', error);
    return NextResponse.json({ error: 'Error processing request' }, { status: 400 });
  }
}
