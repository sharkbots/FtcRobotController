package org.firstinspires.ftc.teamcode.tools.stateMachine;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.View;

public class GraphView extends View {
    private StateMachine stateMachine;
    private int xStep = 200;
    private int yStep = 150;
    private int radius = 40;

    public GraphView(Context context, StateMachine stateMachine) {
        super(context);
        this.stateMachine = stateMachine;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (stateMachine != null) {
            paintStateMachine(canvas);
        }
    }

    private void paintStateMachine(Canvas canvas) {
        Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        paint.setTextSize(30);

        int x = 100;
        int y = 100;

        // Draw states
        for (StateMachine.State state : stateMachine.states) {
            paint.setColor(Color.BLUE);
            canvas.drawCircle(x, y, radius, paint);
            paint.setColor(Color.WHITE);
            canvas.drawText(state.name, x - radius / 2, y + 10, paint);

            // Draw transitions from each state
            int tx = x;
            int ty = y + radius + 20; // Start drawing transitions slightly below the state
            for (StateMachine.State.Transition transition : state.transitions) {
                paint.setColor(Color.RED);
                canvas.drawLine(tx, ty, tx, ty + yStep / 2, paint);
                canvas.drawText("to " + transition.getDestinationState().name, tx - 50, ty + yStep / 2 + 20, paint);
                tx += xStep; // Increment x for next transition
            }
            y += yStep; // Increment y for next state
        }
    }
}
