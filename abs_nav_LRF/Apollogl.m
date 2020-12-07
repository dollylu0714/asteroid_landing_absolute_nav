function [ Out_C0, Out_C1, Out_C2 ] = Apollogl( atf, vtf, rtf, vto, rto, t )
%APOLLOGL : Apollo guidance law
%
Out_C0 = atf - 6*(vtf-vto)./t + 12*(rtf-rto-vto*t)./(t^2);
Out_C1 = -6*atf./t + 30*(vtf-vto)./(t^2) - 48*(rtf-rto-vto*t)./(t^3);
Out_C2 = 6*atf./(t^2) - 24*(vtf-vto)./(t^3) + 36*(rtf-rto-vto*t)./(t^4);

end

