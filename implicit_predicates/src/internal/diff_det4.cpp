#include <implicit_point.h>

#pragma intrinsic(fabs)

int diff_det4_filtered(double p0,
                       double p1,
                       double p2,
                       double p3,
                       double q0,
                       double q1,
                       double q2,
                       double q3,
                       double r0,
                       double r1,
                       double r2,
                       double r3,
                       double s0,
                       double s1,
                       double s2,
                       double s3,
                       double t0,
                       double t1,
                       double t2,
                       double t3)
{
    double a_      = q0 - p0;
    double b_      = q1 - p1;
    double c_      = q2 - p2;
    double d_      = q3 - p3;
    double e_      = r0 - p0;
    double f_      = r1 - p1;
    double g_      = r2 - p2;
    double h_      = r3 - p3;
    double i_      = s0 - p0;
    double j_      = s1 - p1;
    double k_      = s2 - p2;
    double l_      = s3 - p3;
    double m_      = t0 - p0;
    double n_      = t1 - p1;
    double o_      = t2 - p2;
    double p_      = t3 - p3;
    double af      = a_ * f_;
    double be      = b_ * e_;
    double kp      = k_ * p_;
    double lo      = l_ * o_;
    double ce      = c_ * e_;
    double ag      = a_ * g_;
    double jp      = j_ * p_;
    double ln      = l_ * n_;
    double ah      = a_ * h_;
    double de      = d_ * e_;
    double jo      = j_ * o_;
    double kn      = k_ * n_;
    double bg      = b_ * g_;
    double cf      = c_ * f_;
    double ip      = i_ * p_;
    double lm      = l_ * m_;
    double df      = d_ * f_;
    double bh      = b_ * h_;
    double io      = i_ * o_;
    double km      = k_ * m_;
    double ch      = c_ * h_;
    double dg      = d_ * g_;
    double in      = i_ * n_;
    double jm      = j_ * m_;
    double d1      = af - be;
    double d2      = kp - lo;
    double d3      = ce - ag;
    double d4      = jp - ln;
    double d5      = ah - de;
    double d6      = jo - kn;
    double d7      = bg - cf;
    double d8      = ip - lm;
    double d9      = df - bh;
    double d10     = io - km;
    double d11     = ch - dg;
    double d12     = in - jm;
    double term1   = d1 * d2;
    double term2   = d3 * d4;
    double term3   = d5 * d6;
    double term4   = d7 * d8;
    double term5   = d9 * d10;
    double term6   = d11 * d12;
    double r12     = term1 + term2;
    double r34     = term3 + term4;
    double r56     = term5 + term6;
    double r1234   = r12 + r34;
    double r123456 = r1234 + r56;

    double _tmp_fabs;

    double max_var = 0.0;
    if ((_tmp_fabs = fabs(a_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(b_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(c_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(d_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(e_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(f_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(g_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(h_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(i_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(j_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(k_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(l_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(m_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(n_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(o_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(p_)) > max_var) max_var = _tmp_fabs;
    double epsilon  = max_var;
    epsilon        *= epsilon;
    epsilon        *= epsilon;
    epsilon        *= 3.019806626980427e-14;
    if (r123456 > epsilon) return IP_Sign::POSITIVE;
    if (-r123456 > epsilon) return IP_Sign::NEGATIVE;
    return Filtered_Sign::UNCERTAIN;
}

int diff_det4_interval(interval_number p0,
                       interval_number p1,
                       interval_number p2,
                       interval_number p3,
                       interval_number q0,
                       interval_number q1,
                       interval_number q2,
                       interval_number q3,
                       interval_number r0,
                       interval_number r1,
                       interval_number r2,
                       interval_number r3,
                       interval_number s0,
                       interval_number s1,
                       interval_number s2,
                       interval_number s3,
                       interval_number t0,
                       interval_number t1,
                       interval_number t2,
                       interval_number t3)
{
    setFPUModeToRoundUP();
    interval_number a_(q0 - p0);
    interval_number b_(q1 - p1);
    interval_number c_(q2 - p2);
    interval_number d_(q3 - p3);
    interval_number e_(r0 - p0);
    interval_number f_(r1 - p1);
    interval_number g_(r2 - p2);
    interval_number h_(r3 - p3);
    interval_number i_(s0 - p0);
    interval_number j_(s1 - p1);
    interval_number k_(s2 - p2);
    interval_number l_(s3 - p3);
    interval_number m_(t0 - p0);
    interval_number n_(t1 - p1);
    interval_number o_(t2 - p2);
    interval_number p_(t3 - p3);
    interval_number af(a_ * f_);
    interval_number be(b_ * e_);
    interval_number kp(k_ * p_);
    interval_number lo(l_ * o_);
    interval_number ce(c_ * e_);
    interval_number ag(a_ * g_);
    interval_number jp(j_ * p_);
    interval_number ln(l_ * n_);
    interval_number ah(a_ * h_);
    interval_number de(d_ * e_);
    interval_number jo(j_ * o_);
    interval_number kn(k_ * n_);
    interval_number bg(b_ * g_);
    interval_number cf(c_ * f_);
    interval_number ip(i_ * p_);
    interval_number lm(l_ * m_);
    interval_number df(d_ * f_);
    interval_number bh(b_ * h_);
    interval_number io(i_ * o_);
    interval_number km(k_ * m_);
    interval_number ch(c_ * h_);
    interval_number dg(d_ * g_);
    interval_number in(i_ * n_);
    interval_number jm(j_ * m_);
    interval_number d1(af - be);
    interval_number d2(kp - lo);
    interval_number d3(ce - ag);
    interval_number d4(jp - ln);
    interval_number d5(ah - de);
    interval_number d6(jo - kn);
    interval_number d7(bg - cf);
    interval_number d8(ip - lm);
    interval_number d9(df - bh);
    interval_number d10(io - km);
    interval_number d11(ch - dg);
    interval_number d12(in - jm);
    interval_number term1(d1 * d2);
    interval_number term2(d3 * d4);
    interval_number term3(d5 * d6);
    interval_number term4(d7 * d8);
    interval_number term5(d9 * d10);
    interval_number term6(d11 * d12);
    interval_number r12(term1 + term2);
    interval_number r34(term3 + term4);
    interval_number r56(term5 + term6);
    interval_number r1234(r12 + r34);
    interval_number r123456(r1234 + r56);
    setFPUModeToRoundNEAR();

    if (!r123456.signIsReliable()) return Filtered_Sign::UNCERTAIN;
    return r123456.sign();
}

int diff_det4_exact(double p0,
                    double p1,
                    double p2,
                    double p3,
                    double q0,
                    double q1,
                    double q2,
                    double q3,
                    double r0,
                    double r1,
                    double r2,
                    double r3,
                    double s0,
                    double s1,
                    double s2,
                    double s3,
                    double t0,
                    double t1,
                    double t2,
                    double t3)
{
    expansionObject o;
    double          a_[2];
    o.two_Diff(q0, p0, a_);
    double b_[2];
    o.two_Diff(q1, p1, b_);
    double c_[2];
    o.two_Diff(q2, p2, c_);
    double d_[2];
    o.two_Diff(q3, p3, d_);
    double e_[2];
    o.two_Diff(r0, p0, e_);
    double f_[2];
    o.two_Diff(r1, p1, f_);
    double g_[2];
    o.two_Diff(r2, p2, g_);
    double h_[2];
    o.two_Diff(r3, p3, h_);
    double i_[2];
    o.two_Diff(s0, p0, i_);
    double j_[2];
    o.two_Diff(s1, p1, j_);
    double k_[2];
    o.two_Diff(s2, p2, k_);
    double l_[2];
    o.two_Diff(s3, p3, l_);
    double m_[2];
    o.two_Diff(t0, p0, m_);
    double n_[2];
    o.two_Diff(t1, p1, n_);
    double o_[2];
    o.two_Diff(t2, p2, o_);
    double p_[2];
    o.two_Diff(t3, p3, p_);
    double af[8];
    int    af_len = o.Gen_Product(2, a_, 2, f_, af);
    double be[8];
    int    be_len = o.Gen_Product(2, b_, 2, e_, be);
    double kp[8];
    int    kp_len = o.Gen_Product(2, k_, 2, p_, kp);
    double lo[8];
    int    lo_len = o.Gen_Product(2, l_, 2, o_, lo);
    double ce[8];
    int    ce_len = o.Gen_Product(2, c_, 2, e_, ce);
    double ag[8];
    int    ag_len = o.Gen_Product(2, a_, 2, g_, ag);
    double jp[8];
    int    jp_len = o.Gen_Product(2, j_, 2, p_, jp);
    double ln[8];
    int    ln_len = o.Gen_Product(2, l_, 2, n_, ln);
    double ah[8];
    int    ah_len = o.Gen_Product(2, a_, 2, h_, ah);
    double de[8];
    int    de_len = o.Gen_Product(2, d_, 2, e_, de);
    double jo[8];
    int    jo_len = o.Gen_Product(2, j_, 2, o_, jo);
    double kn[8];
    int    kn_len = o.Gen_Product(2, k_, 2, n_, kn);
    double bg[8];
    int    bg_len = o.Gen_Product(2, b_, 2, g_, bg);
    double cf[8];
    int    cf_len = o.Gen_Product(2, c_, 2, f_, cf);
    double ip[8];
    int    ip_len = o.Gen_Product(2, i_, 2, p_, ip);
    double lm[8];
    int    lm_len = o.Gen_Product(2, l_, 2, m_, lm);
    double df[8];
    int    df_len = o.Gen_Product(2, d_, 2, f_, df);
    double bh[8];
    int    bh_len = o.Gen_Product(2, b_, 2, h_, bh);
    double io[8];
    int    io_len = o.Gen_Product(2, i_, 2, o_, io);
    double km[8];
    int    km_len = o.Gen_Product(2, k_, 2, m_, km);
    double ch[8];
    int    ch_len = o.Gen_Product(2, c_, 2, h_, ch);
    double dg[8];
    int    dg_len = o.Gen_Product(2, d_, 2, g_, dg);
    double in[8];
    int    in_len = o.Gen_Product(2, i_, 2, n_, in);
    double jm[8];
    int    jm_len = o.Gen_Product(2, j_, 2, m_, jm);
    double d1[16];
    int    d1_len = o.Gen_Diff(af_len, af, be_len, be, d1);
    double d2[16];
    int    d2_len = o.Gen_Diff(kp_len, kp, lo_len, lo, d2);
    double d3[16];
    int    d3_len = o.Gen_Diff(ce_len, ce, ag_len, ag, d3);
    double d4[16];
    int    d4_len = o.Gen_Diff(jp_len, jp, ln_len, ln, d4);
    double d5[16];
    int    d5_len = o.Gen_Diff(ah_len, ah, de_len, de, d5);
    double d6[16];
    int    d6_len = o.Gen_Diff(jo_len, jo, kn_len, kn, d6);
    double d7[16];
    int    d7_len = o.Gen_Diff(bg_len, bg, cf_len, cf, d7);
    double d8[16];
    int    d8_len = o.Gen_Diff(ip_len, ip, lm_len, lm, d8);
    double d9[16];
    int    d9_len = o.Gen_Diff(df_len, df, bh_len, bh, d9);
    double d10[16];
    int    d10_len = o.Gen_Diff(io_len, io, km_len, km, d10);
    double d11[16];
    int    d11_len = o.Gen_Diff(ch_len, ch, dg_len, dg, d11);
    double d12[16];
    int    d12_len = o.Gen_Diff(in_len, in, jm_len, jm, d12);
    double term1_p[128], *term1 = term1_p;
    int    term1_len = o.Gen_Product_With_PreAlloc(d1_len, d1, d2_len, d2, &term1, 128);
    double term2_p[128], *term2 = term2_p;
    int    term2_len = o.Gen_Product_With_PreAlloc(d3_len, d3, d4_len, d4, &term2, 128);
    double term3_p[128], *term3 = term3_p;
    int    term3_len = o.Gen_Product_With_PreAlloc(d5_len, d5, d6_len, d6, &term3, 128);
    double term4_p[128], *term4 = term4_p;
    int    term4_len = o.Gen_Product_With_PreAlloc(d7_len, d7, d8_len, d8, &term4, 128);
    double term5_p[128], *term5 = term5_p;
    int    term5_len = o.Gen_Product_With_PreAlloc(d9_len, d9, d10_len, d10, &term5, 128);
    double term6_p[128], *term6 = term6_p;
    int    term6_len = o.Gen_Product_With_PreAlloc(d11_len, d11, d12_len, d12, &term6, 128);
    double r12_p[128], *r12 = r12_p;
    int    r12_len = o.Gen_Sum_With_PreAlloc(term1_len, term1, term2_len, term2, &r12, 128);
    double r34_p[128], *r34 = r34_p;
    int    r34_len = o.Gen_Sum_With_PreAlloc(term3_len, term3, term4_len, term4, &r34, 128);
    double r56_p[128], *r56 = r56_p;
    int    r56_len = o.Gen_Sum_With_PreAlloc(term5_len, term5, term6_len, term6, &r56, 128);
    double r1234_p[128], *r1234 = r1234_p;
    int    r1234_len = o.Gen_Sum_With_PreAlloc(r12_len, r12, r34_len, r34, &r1234, 128);
    double r123456_p[128], *r123456 = r123456_p;
    int    r123456_len = o.Gen_Sum_With_PreAlloc(r1234_len, r1234, r56_len, r56, &r123456, 128);

    double return_value = r123456[r123456_len - 1];
    if (r123456_p != r123456) free(r123456);
    if (r1234_p != r1234) free(r1234);
    if (r56_p != r56) free(r56);
    if (r34_p != r34) free(r34);
    if (r12_p != r12) free(r12);
    if (term6_p != term6) free(term6);
    if (term5_p != term5) free(term5);
    if (term4_p != term4) free(term4);
    if (term3_p != term3) free(term3);
    if (term2_p != term2) free(term2);
    if (term1_p != term1) free(term1);

    if (return_value > 0) return IP_Sign::POSITIVE;
    if (return_value < 0) return IP_Sign::NEGATIVE;
    if (return_value == 0) return IP_Sign::ZERO;
    return IP_Sign::UNDEFINED;
}

int diff_det4(double p0,
              double p1,
              double p2,
              double p3,
              double q0,
              double q1,
              double q2,
              double q3,
              double r0,
              double r1,
              double r2,
              double r3,
              double s0,
              double s1,
              double s2,
              double s3,
              double t0,
              double t1,
              double t2,
              double t3)
{
    int ret;
#ifdef IMPLICIT_PREDICATES_STAGE_STATS
    semi_static_filter_stage++;
#endif
    ret = diff_det4_filtered(p0, p1, p2, p3, q0, q1, q2, q3, r0, r1, r2, r3, s0, s1, s2, s3, t0, t1, t2, t3);
    if (ret != Filtered_Sign::UNCERTAIN) return ret;

#ifdef IMPLICIT_PREDICATES_STAGE_STATS
    interval_arithmetic_stage++;
#endif
    ret = diff_det4_interval(p0, p1, p2, p3, q0, q1, q2, q3, r0, r1, r2, r3, s0, s1, s2, s3, t0, t1, t2, t3);
    if (ret != Filtered_Sign::UNCERTAIN) return ret;

#ifdef IMPLICIT_PREDICATES_STAGE_STATS
    exact_computation_stage++;
#endif
    return diff_det4_exact(p0, p1, p2, p3, q0, q1, q2, q3, r0, r1, r2, r3, s0, s1, s2, s3, t0, t1, t2, t3);
}