function [a_wf,q_wf] = Base2World(a,q,Rin_world,pin_world)

    a_wf = Rin_world*a;
    temp = Rin_world*q;
    q_wf = temp + pin_world;
    
end