function derivative_a = d_Hamilton_a_b(~,b)

derivative_a=[b(1),-b(2),-b(3),-b(4);
                b(2),b(1),b(4),-b(3);
                b(3),-b(4),b(1),b(2);
                b(4),b(3),-b(2),b(1)];

end