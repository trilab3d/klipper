// Cartesian XE kinematics - X osa s rotujícím extruderem
// Dva X motory: X_left sleduje jen X, X_right sleduje X - E
// X_left = X
// X_right = X - rotation_ratio * E
// Rozdíl pozic = rotation_ratio * E (způsobuje rotaci)
//
// Copyright (C) 2025
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible, container_of
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // struct trapq, move_get_coord

struct cartesian_xe_stepper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *extruder_sk;  // Extruder stepper kinematics (s PA)
    struct trapq *extruder_tq;               // Extruder trapq
    struct trapq *xyz_tq;                    // XYZ trapq (uloženo před změnou)
    double rotation_ratio;                   // Násobitel pro extruder (+1.0 nebo -1.0)
    char base_axis;                          // 'x', 'y', nebo 'z'
    double last_xyz_pos;                     // Cache poslední X pozice
};

// Najdi extruder pozici v daném čase (s pressure advance!)
static double
get_extruder_position(struct stepper_kinematics *extruder_sk,
                      struct trapq *extruder_tq, double print_time)
{
    if (!extruder_sk || !extruder_tq)
        return 0.;

    // Najdi odpovídající move v extruder trapq
    struct move *m;
    list_for_each_entry(m, &extruder_tq->moves, node) {
        double move_end = m->print_time + m->move_t;
        if (print_time < m->print_time) {
            // Před začátkem pohybu
            return m->start_pos.x;
        }
        if (print_time <= move_end) {
            // Print_time je uvnitř tohoto pohybu
            double move_time = print_time - m->print_time;
            // Zavolej extruder calc_position (která zahrnuje PA!)
            return extruder_sk->calc_position_cb(extruder_sk, m, move_time);
        }
    }

    // Print_time je za všemi pohyby
    if (!list_empty(&extruder_tq->moves)) {
        struct move *last = list_last_entry(&extruder_tq->moves, struct move, node);
        return extruder_sk->calc_position_cb(extruder_sk, last, last->move_t);
    }

    return 0.;
}

static double
cartesian_xe_calc_position(struct stepper_kinematics *sk, struct move *m,
                           double move_time)
{
    struct cartesian_xe_stepper *cxe = container_of(
        sk, struct cartesian_xe_stepper, sk);

    double base_pos = 0.;
    double print_time = m->print_time + move_time;

    // Pro rotation_ratio == 0 (X_left): Move je vždy z XYZ trapq
    // Pro rotation_ratio != 0 (X_right): sk->tq ukazuje na E trapq!
    //   Musíme získat X pozici z xyz_tq podle print_time

    if (cxe->rotation_ratio == 0.) {
        // X_left - standardní chování
        struct coord c = move_get_coord(m, move_time);
        switch (cxe->base_axis) {
            case 'x': base_pos = c.x; break;
            case 'y': base_pos = c.y; break;
            case 'z': base_pos = c.z; break;
            default: base_pos = 0.; break;
        }
    } else {
        // stepper_xe - move je z E trapq, musíme najít X pozici z xyz_tq
        if (cxe->xyz_tq && !list_empty(&cxe->xyz_tq->moves)) {
            int found = 0;
            struct move *xyz_m;
            list_for_each_entry(xyz_m, &cxe->xyz_tq->moves, node) {
                double xyz_end = xyz_m->print_time + xyz_m->move_t;
                if (print_time < xyz_m->print_time) {
                    // Před začátkem tohoto move
                    base_pos = xyz_m->start_pos.x;
                    found = 1;
                    break;
                }
                if (print_time <= xyz_end) {
                    // Uvnitř tohoto move
                    double xyz_time = print_time - xyz_m->print_time;
                    struct coord c = move_get_coord(xyz_m, xyz_time);
                    base_pos = c.x;
                    found = 1;
                    break;
                }
            }
            if (!found) {
                // print_time je za všemi moves - použij konec posledního
                struct move *last = list_last_entry(&cxe->xyz_tq->moves,
                                                     struct move, node);
                struct coord c = move_get_coord(last, last->move_t);
                base_pos = c.x;
            }
        } else {
            // Žádné XYZ moves - použij poslední známou pozici
            base_pos = cxe->last_xyz_pos;
        }
        cxe->last_xyz_pos = base_pos;
    }

    // Získej extruder pozici (s pressure advance!)
    double extruder_pos = get_extruder_position(cxe->extruder_sk,
                                                 cxe->extruder_tq,
                                                 print_time);

    // Kombinuj obě pozice: X_motor = X + rotation_ratio * E
    return base_pos + cxe->rotation_ratio * extruder_pos;
}

struct stepper_kinematics * __visible
cartesian_xe_stepper_alloc(char base_axis)
{
    struct cartesian_xe_stepper *cxe = malloc(sizeof(*cxe));
    memset(cxe, 0, sizeof(*cxe));
    cxe->sk.calc_position_cb = cartesian_xe_calc_position;
    cxe->base_axis = base_axis;
    cxe->extruder_sk = NULL;
    cxe->rotation_ratio = 0.;

    // Active flags podle base_axis
    if (base_axis == 'x')
        cxe->sk.active_flags = AF_X;
    else if (base_axis == 'y')
        cxe->sk.active_flags = AF_Y;
    else if (base_axis == 'z')
        cxe->sk.active_flags = AF_Z;

    return &cxe->sk;
}

void __visible
cartesian_xe_set_extruder_sk(struct stepper_kinematics *sk,
                              struct stepper_kinematics *extruder_sk,
                              double rotation_ratio)
{
    struct cartesian_xe_stepper *cxe = container_of(
        sk, struct cartesian_xe_stepper, sk);
    cxe->extruder_sk = extruder_sk;
    cxe->rotation_ratio = rotation_ratio;

    // Ulož extruder trapq
    if (extruder_sk && extruder_sk->tq)
        cxe->extruder_tq = extruder_sk->tq;

    // Ulož XYZ trapq
    cxe->xyz_tq = sk->tq;

    // Pro X_right (rotation_ratio != 0): nastav sk->tq na E trapq
    // A použij gen_steps_post_active aby generoval i pro XYZ moves
    if (rotation_ratio != 0. && cxe->extruder_tq) {
        sk->tq = cxe->extruder_tq;  // Sleduje E trapq

        // ⭐ KLÍČ: Nastav aby generoval kroky i daleko po E moves
        // Tím způsobíme že itersolve bude pokračovat i když E move skončí
        // ale XYZ move probíhá
        sk->gen_steps_post_active = 999999.;  // Prakticky nekonečno
        sk->gen_steps_pre_active = 999999.;

        sk->active_flags |= AF_X;
    }
}
