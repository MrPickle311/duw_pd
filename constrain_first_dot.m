function result = constrain_first_dot(q,t,rot_pairs,bodies,body0)

% tutaj niezerowe sa tylko funkcje z wiezow kierujacych

lk1 = sqrt(17)/5;
ak1 = 0.1;
wk1 = 0.1;
fi_k1 = 0.0;

lk2 = sqrt(26)/5;
ak2 = 0.1;
wk2 = 0.1;
fi_k2 = 0.0;

func1_dot = @(t_i) ak1*wk1*cos(t_i*wk1+fi_k1);
func2_dot = @(t_i) ak2*wk2*cos(t_i*wk2+fi_k2);

result(29,1) = func1_dot(t);
result(30,1) = func2_dot(t);

% for l=1:length(Wiezy)
%     if(lower(Wiezy(l).typ(1)) == 'd')
%         if(lower(Wiezy(l).klasa(1)) == 'o')
%             Prawe(m) = eval(Wiezy(l).dotfodt);
%             m=m+1;
%         elseif(lower(Wiezy(l).klasa(1)) == 'p')
%             Prawe(m) = eval(Wiezy(l).dotfodt);
%             m=m+1;
%         end
%     end
%     if(lower(Wiezy(l).typ(1)) == 'k')
%         if(lower(Wiezy(l).klasa(1)) == 'o')
%             % Prawe(m:(m+1), 1) = zeros(2,1);
%             m=m+2;
%         elseif(lower(Wiezy(l).klasa(1)) == 'p')
%             % Prawe(m:(m+1), 1) = zeros(2,1);
%             m=m+2;
%         end
%     end
% end

end

